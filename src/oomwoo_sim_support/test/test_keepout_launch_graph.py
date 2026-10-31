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

"""ROS2 Jazzy launch graph checks: no Gazebo spawn or motor activity."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from oomwoo_sim_support.keepout_config import _pgm
import yaml

try:
    from launch import LaunchContext
    from launch.actions import OpaqueFunction, SetLaunchConfiguration
except ImportError:
    LaunchContext = None


@unittest.skipIf(LaunchContext is None, 'ROS2 Jazzy launch is unavailable')
class KeepoutLaunchGraphTests(unittest.TestCase):
    """Evaluate launch argument branching without starting the sim."""

    def setUp(self):
        self.pkg = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location(
            'oomwoo_sim_keepout_launch', self.pkg / 'launch/sim_bringup.launch.py')
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.description = self.module.generate_launch_description()
        self.opaque = [a for a in self.description.entities
                       if isinstance(a, OpaqueFunction)]
        self.assertEqual(len(self.opaque), 1)

    def evaluate(self, mask, map_file=None):
        ctx = LaunchContext()
        for key, value in {
            'robot_model': 'oomwoo_one',
            'keepout_mask': mask,
            'map': str(map_file or self.pkg / 'maps/test_room.yaml'),
            'nav2_params_file': str(self.pkg / 'config/nav2_params.yaml'),
            'x_pose': '0.0', 'y_pose': '0.0', 'yaw': '0.0',
            'world': 'unused', 'with_nav': 'true', 'gui': 'false',
        }.items():
            ctx.launch_configurations[key] = value
        return ctx, self.opaque[0].execute(ctx)

    def test_default_graph_does_not_enable_filters(self):
        _, actions = self.evaluate('')
        self.assertFalse(any(isinstance(a, SetLaunchConfiguration)
                             for a in actions))
        self.assertEqual(len(actions), 6)

    def test_optin_graph_generates_both_costmap_filters(self):
        with tempfile.TemporaryDirectory() as tmp_name:
            tmp = Path(tmp_name)
            mapdata = yaml.safe_load(
                (self.pkg / 'maps/test_room.yaml').read_text())
            width, height, _ = _pgm(self.pkg / 'maps' / mapdata['image'])
            pix = bytearray([255] * (width * height))
            pix[(height // 2) * width + width // 2] = 0
            (tmp / 'mask.pgm').write_bytes(
                f'P5\n{width} {height}\n255\n'.encode() + pix)
            mapdata['image'] = 'mask.pgm'
            (tmp / 'mask.yaml').write_text(yaml.safe_dump(mapdata))
            ctx, actions = self.evaluate(str(tmp / 'mask.yaml'))
            setters = [a for a in actions
                       if isinstance(a, SetLaunchConfiguration)]
            self.assertEqual(len(setters), 1)
            setters[0].execute(ctx)
            params = yaml.safe_load(Path(
                ctx.launch_configurations['nav2_params_file']).read_text())
            for name in ('global_costmap', 'local_costmap'):
                self.assertEqual(
                    params[name][name]['ros__parameters']['filters'],
                    ['keepout_filter'])
                self.assertIn('keepout_inflation',
                              params[name][name]['ros__parameters']['plugins'])
            self.assertEqual(len(actions), 11)

    def test_mask_server_uses_derived_unknown_mask(self):
        with tempfile.TemporaryDirectory() as tmp_name:
            tmp = Path(tmp_name)
            metadata = yaml.safe_load((self.pkg / 'maps/test_room.yaml').read_text())
            width, height, pixels = _pgm(self.pkg / 'maps' / metadata['image'])
            source = bytearray(pixels)
            source[39 * width + 72] = 128
            header = f'P5\n{width} {height}\n255\n'.encode()
            (tmp / 'source.pgm').write_bytes(header + source)
            metadata['image'] = 'source.pgm'
            (tmp / 'source.yaml').write_text(yaml.safe_dump(metadata))
            mask = bytearray([255] * (width * height))
            mask[70 * width + 70] = 0
            (tmp / 'mask.pgm').write_bytes(header + mask)
            metadata['image'] = 'mask.pgm'
            (tmp / 'mask.yaml').write_text(yaml.safe_dump(metadata))
            actual_node = self.module.Node
            recorded = []

            def node(**kwargs):
                recorded.append(kwargs)
                return actual_node(**kwargs)

            with patch.object(self.module, 'Node', side_effect=node):
                ctx, actions = self.evaluate(str(tmp / 'mask.yaml'), tmp / 'source.yaml')
            setter = next(a for a in actions if isinstance(a, SetLaunchConfiguration))
            setter.execute(ctx)
            overlay = ctx.launch_configurations['nav2_params_file']
            params = yaml.safe_load(Path(overlay).read_text())
            served = Path(params['oomwoo_keepout_mask_server']['ros__parameters']['yaml_filename'])
            self.assertNotEqual(served, tmp / 'mask.yaml')
            server = next(n for n in recorded if n.get('name') == 'oomwoo_keepout_mask_server')
            self.assertEqual(server['parameters'][0], overlay)
            self.assertNotIn('yaml_filename', server['parameters'][1])
            self.assertEqual(server['parameters'][1]['topic_name'], '/oomwoo/keepout_mask')
            info = yaml.safe_load(served.read_text())
            self.assertEqual(_pgm(served.parent / info['image'])[2].count(0), 2)
            self.assertEqual(len(actions), 11)
