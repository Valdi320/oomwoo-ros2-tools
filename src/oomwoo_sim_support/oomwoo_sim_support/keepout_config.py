# Copyright 2026 Valdi320
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Opt-in Nav2 KeepoutFilter overlay for the OOMWOO headless simulator.

Does not modify source params, does not drive hardware, and does not
authorize reactive cmd_vel coverage. Python 3.10+/ROS2 Jazzy.
"""
from __future__ import annotations

import math
from pathlib import Path
import shutil
import struct
import tempfile

import yaml

MASK_TOPIC = '/oomwoo/keepout_mask'
INFO_TOPIC = '/oomwoo/keepout_filter_info'


def _source_unknown_pixels(source: dict, pixels: bytes) -> bytes:
    """Decode unknown P5 pixels with Nav2's map modes, without changing the map.

    P5 has no alpha channel. Scale therefore has no unknown intermediate
    values; raw values above 100 are unknown. Trinary uses float32 intensity
    and comparisons, as the installed Nav2 map_io implementation does.
    """
    mode = source.get('mode', 'trinary')
    if mode == 'raw':
        return bytes(value > 100 for value in pixels)
    if mode == 'scale':
        return bytes(len(pixels))
    if mode != 'trinary':
        raise ValueError('invalid source map mode')
    free, occupied = source.get('free_thresh'), source.get('occupied_thresh')
    negate = source.get('negate')
    if (any(type(value) not in (int, float) or not math.isfinite(value)
            or not 0 <= value <= 1 for value in (free, occupied))
            or type(negate) not in (int, bool)):
        raise ValueError('invalid source map occupancy metadata')

    def f32(value):
        return struct.unpack('<f', struct.pack('<f', value))[0]

    free, occupied = f32(free), f32(occupied)
    table = bytearray(256)
    for value in range(256):
        intensity = f32(value / 255.)
        probability = intensity if negate else f32(1. - intensity)
        table[value] = not (probability >= occupied or probability <= free)
    return pixels.translate(bytes(table))


def _metadata(path: Path) -> dict:
    """Read map metadata with no escaping image path and finite geometry."""
    if not path.is_file():
        raise ValueError(f'missing map YAML: {path}')
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError('invalid YAML map')
    if not all(k in data for k in ('image', 'resolution', 'origin')):
        raise ValueError('missing map metadata fields')
    image = Path(str(data['image']))
    if image.is_absolute() or '..' in image.parts:
        raise ValueError('unsafe image path')
    target = (path.parent / image).resolve()
    if path.parent.resolve() not in target.parents:
        raise ValueError('image escapes map directory')
    data['_image_path'] = target
    r, origin = data['resolution'], data['origin']
    if (isinstance(r, bool) or not isinstance(r, (int, float))
            or not math.isfinite(r) or r <= 0
            or not isinstance(origin, (list, tuple)) or len(origin) != 3
            or any(isinstance(x, bool) or not isinstance(x, (int, float))
                   or not math.isfinite(x) for x in origin)
            or abs(origin[2]) > 1e-9):
        raise ValueError('nonfinite or rotated map geometry')
    return data


def _pgm(path: Path) -> tuple[int, int, bytes]:
    """Read bounded P5 (8-bit), including comments in the text header."""
    if not path.is_file() or path.stat().st_size > 20_000_128:
        raise ValueError('missing or oversized PGM')
    raw = path.read_bytes()
    pos = 0

    def token():
        nonlocal pos
        while pos < len(raw):
            if raw[pos] in b' \t\r\n':
                pos += 1
            elif raw[pos] == 35:
                end = raw.find(b'\n', pos)
                pos = len(raw) if end < 0 else end + 1
            else:
                break
        start = pos
        while pos < len(raw) and raw[pos] not in b' \t\r\n':
            pos += 1
        if start == pos:
            raise ValueError('truncated PGM header')
        return raw[start:pos]

    if token() != b'P5':
        raise ValueError('only P5 PGM supported')
    try:
        width, height, maxval = int(token()), int(token()), int(token())
    except ValueError as ex:
        raise ValueError('invalid PGM dimensions') from ex
    if (width < 1 or height < 1 or width * height > 20_000_000
            or maxval != 255 or pos >= len(raw)
            or raw[pos] not in b' \t\r\n'):
        raise ValueError('invalid PGM header')
    pixels = raw[pos + 1:]
    if len(pixels) != width * height:
        raise ValueError('truncated or excess PGM pixel data')
    return width, height, pixels


def build_keepout_overlay(
    nav_params_file: str | Path,
    map_yaml: str | Path,
    mask_yaml: str | Path,
    *,
    output_dir: str | Path,
) -> tuple[Path, int]:
    """Validate mask/map pair and write a NEW Nav2 params file, never overwrite."""
    source = _metadata(Path(map_yaml).resolve())
    mask = _metadata(Path(mask_yaml).resolve())
    sw, sh, source_pixels = _pgm(source['_image_path'])
    mw, mh, pixels = _pgm(mask['_image_path'])
    if (sw, sh) != (mw, mh):
        raise ValueError('mask dimensions differ from map')
    if (abs(source['resolution'] - mask['resolution']) > 1e-9
            or any(abs(float(x) - float(y)) > 1e-8
                   for x, y in zip(source['origin'], mask['origin']))):
        raise ValueError('mask origin/resolution differ from map')
    if (mask.get('mode') != 'trinary' or mask.get('negate') != 0
            or mask.get('occupied_thresh') != 0.65
            or mask.get('free_thresh') != 0.25):
        raise ValueError('mask must be exact trinary, non-negated 0/255')
    if any(v not in (0, 255) for v in pixels):
        raise ValueError('ambiguous gray or unknown mask pixels')
    forbidden = pixels.count(0)
    if forbidden == 0 or forbidden == len(pixels):
        raise ValueError('mask requires both forbidden and free cells')
    unknown = _source_unknown_pixels(source, source_pixels)
    if any(unknown):
        # Keep the original binary contract. Unknown terrain is blocked rather
        # than copied as free by KeepoutFilter's NO_INFORMATION overwrite rule.
        pixels = bytes(0 if mark else value for value, mark in zip(pixels, unknown))
        forbidden = pixels.count(0)
        if forbidden == len(pixels):
            raise ValueError('mask requires free cells after unknown-map protection')

    nav = yaml.safe_load(Path(nav_params_file).read_text(encoding='utf-8'))
    if not isinstance(nav, dict):
        raise ValueError('invalid Nav2 parameters')
    for costmap in ('global_costmap', 'local_costmap'):
        try:
            params = nav[costmap][costmap]['ros__parameters']
            radius = float(params['robot_radius'])
            resolution = float(params['resolution'])
        except (KeyError, TypeError, ValueError) as ex:
            raise ValueError(f'missing {costmap} robot radius/resolution') from ex
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError('invalid robot radius')
        if (isinstance(params['resolution'], bool)
                or not math.isfinite(resolution) or resolution <= 0):
            raise ValueError('invalid costmap resolution')
        plugins = params.get('plugins',
                             ['static_layer', 'obstacle_layer', 'inflation_layer'])
        if not isinstance(plugins, list) or any(not isinstance(p, str) for p in plugins):
            raise ValueError('invalid costmap plugins')
        if (params.get('filters')
                or any(name in params or name in plugins for name in
                       ('keepout_filter', 'keepout_inflation'))):
            raise ValueError(f'{costmap} already has filters or keepout plugin blocks')
        params['filters'] = ['keepout_filter']
        params['plugins'] = [*plugins, 'keepout_inflation']
        params['keepout_filter'] = {
            'plugin': 'nav2_costmap_2d::KeepoutFilter',
            'enabled': True,
            'filter_info_topic': INFO_TOPIC,
            'override_lethal_cost': False,
            'transform_tolerance': 0.3,
        }
        # Inflate only the served virtual mask. Inflating the merged master
        # also adds inscribed costs around ordinary furniture and defeats the
        # contact-tolerant profile. The container must not turn unknown floor
        # into free space when the binary mask is merged back into the master.
        params['keepout_inflation'] = {
            'plugin': 'nav2_costmap_2d::PluginContainerLayer',
            'enabled': True,
            'combination_method': 2,  # MaxWithoutUnknownOverwrite
            'plugins': ['mask', 'inflation'],
            'mask': {
                'plugin': 'nav2_costmap_2d::StaticLayer',
                'enabled': True,
                'map_topic': MASK_TOPIC,
                'map_subscribe_transient_local': True,
                'subscribe_to_updates': False,
                'use_maximum': False,
                'track_unknown_space': True,
                'trinary_costmap': True,
                'lethal_cost_threshold': 100,
            },
            'inflation': {
                'plugin': 'nav2_costmap_2d::InflationLayer',
                'enabled': True,
                # Support the soft field beyond the command guard's margin.
                # Navfn interpolates between cells; the native comparison needs
                # two more cells to retain its existing 0.24 m disk audit.
                # This is not a proof of clearance; the command guard remains.
                'inflation_radius': round(radius + 0.06 + 2 * resolution, 4),
                'cost_scaling_factor': 5.0,
            },
        }

    target = Path(output_dir)
    if not target.is_dir():
        raise ValueError('output directory must exist')
    outfile = target / 'nav2_params_with_keepout.yaml'
    owned_mask = None
    try:
        served_mask = Path(mask_yaml).resolve()
        if any(unknown):
            # Own a separate directory so failed/concurrent builds can remove
            # only their assets. The launch's existing scratch cleanup owns it.
            owned_mask = Path(tempfile.mkdtemp(prefix='protected_mask_', dir=target))
            with (owned_mask / 'mask.pgm').open('xb') as stream:
                stream.write(f'P5\n{mw} {mh}\n255\n'.encode() + pixels)
            served_mask = owned_mask / 'mask.yaml'
            metadata = {key: value for key, value in mask.items()
                        if not key.startswith('_')}
            metadata['image'] = 'mask.pgm'
            with served_mask.open('x', encoding='utf-8') as stream:
                stream.write(yaml.safe_dump(metadata, sort_keys=False))
        server = nav.setdefault('oomwoo_keepout_mask_server', {}).setdefault(
            'ros__parameters', {})
        server['yaml_filename'] = str(served_mask)
        # Serialize before exclusively acquiring the output name. Preserve a
        # concurrent winner, including dangling symlinks, exactly as before.
        payload = yaml.safe_dump(nav, sort_keys=False)
        with outfile.open('x', encoding='utf-8') as stream:
            stream.write(payload)
    except FileExistsError as ex:
        if owned_mask is not None:
            shutil.rmtree(owned_mask)
        raise ValueError('refuse to overwrite existing overlay') from ex
    except Exception:
        if owned_mask is not None:
            shutil.rmtree(owned_mask)
        raise
    return outfile, forbidden
