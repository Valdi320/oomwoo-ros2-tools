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

"""Pure-Python tests for the opt-in simulator keepout overlay."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from oomwoo_sim_support.keepout_config import _pgm, build_keepout_overlay
import yaml


class KeepoutConfigTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.map = self.root / 'map.yaml'
        self.mask = self.root / 'mask.yaml'
        self.nav = self.root / 'nav.yaml'
        self.output = self.root / 'output'
        self.output.mkdir()
        self.pgm('map.pgm', bytes([255] * 12))
        self.pgm('mask.pgm', bytes([255] * 5 + [0] + [255] * 6))
        for path, image in ((self.map, 'map.pgm'), (self.mask, 'mask.pgm')):
            path.write_text(yaml.safe_dump({
                'image': image, 'mode': 'trinary', 'resolution': .05,
                'origin': [-.1, -.2, 0], 'negate': 0,
                'occupied_thresh': .65, 'free_thresh': .25,
            }), encoding='utf-8')
        self.nav.write_text(yaml.safe_dump({
            n: {n: {'ros__parameters': {
                'robot_radius': .1745,
                'resolution': .05 if n == 'global_costmap' else .02,
                'inflation_layer': {'inflation_radius': .10},
                'plugins': ['static_layer', 'inflation_layer'],
            }}} for n in ('global_costmap', 'local_costmap')
        }), encoding='utf-8')

    def pgm(self, name, pixels, width=4, height=3):
        (self.root / name).write_bytes(
            f'P5\n{width} {height}\n255\n'.encode() + pixels)

    def build(self):
        return build_keepout_overlay(self.nav, self.map, self.mask,
                                     output_dir=self.output)

    def test_both_costmaps_and_unchanged_stock(self):
        original = self.nav.read_bytes()
        out, n = self.build()
        self.assertEqual(n, 1)
        self.assertEqual(self.nav.read_bytes(), original)
        data = yaml.safe_load(out.read_text())
        for name in ('global_costmap', 'local_costmap'):
            params = data[name][name]['ros__parameters']
            self.assertEqual(params['filters'],
                             ['keepout_filter'])
            self.assertEqual(params['keepout_filter']['filter_info_topic'],
                             '/oomwoo/keepout_filter_info')
            container = params['keepout_inflation']
            self.assertEqual(container['plugin'], 'nav2_costmap_2d::PluginContainerLayer')
            self.assertEqual(container['combination_method'], 2)
            self.assertEqual(container['mask']['map_topic'], '/oomwoo/keepout_mask')
            expected_radius = .3345 if name == 'global_costmap' else .2745
            self.assertEqual(container['inflation']['inflation_radius'], expected_radius)
            self.assertEqual(params['inflation_layer']['inflation_radius'], .10)
            self.assertEqual(params['plugins'],
                             ['static_layer', 'inflation_layer', 'keepout_inflation'])

    def test_invalid_costmap_resolution_cannot_publish_an_overlay(self):
        original = self.nav.read_bytes()
        for value in (None, 0, -1, True, float('nan')):
            with self.subTest(resolution=value):
                nav = yaml.safe_load(original)
                nav['local_costmap']['local_costmap']['ros__parameters']['resolution'] = value
                self.nav.write_text(yaml.safe_dump(nav))
                with self.assertRaisesRegex(ValueError, 'resolution'):
                    self.build()
                self.assertFalse((self.output / 'nav2_params_with_keepout.yaml').exists())

    def test_dimensions_differ_fail_closed(self):
        self.pgm('mask.pgm', bytes([255] * 19 + [0]), 5, 4)
        with self.assertRaisesRegex(ValueError, 'dimensions'):
            self.build()

    def test_origin_differ_fail_closed(self):
        meta = yaml.safe_load(self.mask.read_text())
        meta['origin'][0] += .05
        self.mask.write_text(yaml.safe_dump(meta))
        with self.assertRaisesRegex(ValueError, 'origin/resolution'):
            self.build()

    def test_unknown_gray_mask_fail_closed(self):
        self.pgm('mask.pgm', bytes([0, 127] + [255] * 10))
        with self.assertRaisesRegex(ValueError, 'ambiguous'):
            self.build()

    def test_blank_and_total_mask_fail_closed(self):
        for pixels in (bytes([255] * 12), bytes([0] * 12)):
            self.pgm('mask.pgm', pixels)
            with self.assertRaisesRegex(ValueError, 'both forbidden and free'):
                self.build()

    def test_truncated_pgm_fail_closed(self):
        self.pgm('mask.pgm', bytes([255] * 4))
        with self.assertRaisesRegex(ValueError, 'truncated'):
            self.build()

    def test_unsafe_image_path_fail_closed(self):
        meta = yaml.safe_load(self.mask.read_text())
        meta['image'] = '../mask.pgm'
        self.mask.write_text(yaml.safe_dump(meta))
        with self.assertRaisesRegex(ValueError, 'unsafe image path'):
            self.build()

    def test_existing_filter_not_overridden(self):
        params = yaml.safe_load(self.nav.read_text())
        params['local_costmap']['local_costmap']['ros__parameters']['filters'] = ['other']
        self.nav.write_text(yaml.safe_dump(params))
        with self.assertRaisesRegex(ValueError, 'already has filters'):
            self.build()

    def test_no_overwrite_derived_yaml(self):
        self.build()
        with self.assertRaisesRegex(ValueError, 'refuse to overwrite'):
            self.build()

    def test_existing_plugin_blocks_are_not_overridden(self):
        original = self.nav.read_bytes()
        for costmap in ('global_costmap', 'local_costmap'):
            for name in ('keepout_filter', 'keepout_inflation'):
                with self.subTest(costmap=costmap, plugin=name):
                    nav = yaml.safe_load(original)
                    params = nav[costmap][costmap]['ros__parameters']
                    # A valid ordinary layer can use either name already.
                    params['plugins'].append(name)
                    params[name] = {
                        'plugin': 'nav2_costmap_2d::StaticLayer',
                        'map_topic': '/custom_map',
                        'enabled': True,
                    }
                    self.nav.write_text(yaml.safe_dump(nav))
                    before = self.nav.read_bytes()
                    output = self.output / f'{costmap}_{name}'
                    output.mkdir()
                    with self.assertRaisesRegex(
                            ValueError, 'already has filters or keepout plugin blocks'):
                        build_keepout_overlay(self.nav, self.map, self.mask,
                                              output_dir=output)
                    self.assertEqual(self.nav.read_bytes(), before)
                    self.assertFalse((output / 'nav2_params_with_keepout.yaml').exists())

    def test_unrelated_plugin_blocks_are_preserved(self):
        nav = yaml.safe_load(self.nav.read_bytes())
        block = {'plugin': 'nav2_costmap_2d::StaticLayer',
                 'map_topic': '/custom_map', 'enabled': True}
        for costmap in ('global_costmap', 'local_costmap'):
            params = nav[costmap][costmap]['ros__parameters']
            params['plugins'].append('custom_static_layer')
            params['custom_static_layer'] = block.copy()
        self.nav.write_text(yaml.safe_dump(nav))
        before = self.nav.read_bytes()
        out, _ = self.build()
        actual = yaml.safe_load(out.read_bytes())
        for costmap in ('global_costmap', 'local_costmap'):
            params = actual[costmap][costmap]['ros__parameters']
            self.assertEqual(params['custom_static_layer'], block)
            self.assertEqual(params['plugins'],
                             nav[costmap][costmap]['ros__parameters']['plugins']
                             + ['keepout_inflation'])
        self.assertEqual(self.nav.read_bytes(), before)

    def test_missing_source_file_fails_closed(self):
        self.map.unlink()
        with self.assertRaisesRegex(ValueError, 'missing map YAML'):
            self.build()

    def test_concurrent_output_created_during_serialization_is_preserved(self):
        outfile = self.output / 'nav2_params_with_keepout.yaml'
        winner = b'another launch already owns this output\n'
        real_dump = yaml.safe_dump

        def concurrent_writer(*args, **kwargs):
            outfile.write_bytes(winner)
            return real_dump(*args, **kwargs)

        with patch('oomwoo_sim_support.keepout_config.yaml.safe_dump',
                   side_effect=concurrent_writer):
            with self.assertRaisesRegex(ValueError, 'refuse to overwrite'):
                self.build()
        self.assertEqual(outfile.read_bytes(), winner)

    def test_dangling_output_symlink_does_not_create_its_target(self):
        outfile = self.output / 'nav2_params_with_keepout.yaml'
        victim = self.root / 'unrelated-output.yaml'
        outfile.symlink_to(victim)
        with self.assertRaisesRegex(ValueError, 'refuse to overwrite'):
            self.build()
        self.assertTrue(outfile.is_symlink())
        self.assertFalse(victim.exists())

    def test_serialization_failure_leaves_no_output_file(self):
        outfile = self.output / 'nav2_params_with_keepout.yaml'
        original = self.nav.read_bytes()
        with patch('oomwoo_sim_support.keepout_config.yaml.safe_dump',
                   side_effect=ValueError('cannot serialize parameters')):
            with self.assertRaisesRegex(ValueError, 'cannot serialize'):
                self.build()
        self.assertFalse(outfile.exists())
        self.assertEqual(self.nav.read_bytes(), original)

    def test_unknown_source_union_is_binary_without_input_mutation(self):
        pixels = [255] * 12
        pixels[1] = pixels[5] = 128  # outside and inside original forbidden mask
        pixels[7] = 205  # known free at the actual source threshold .25
        self.pgm('map.pgm', bytes(pixels))
        originals = {p: p.read_bytes() for p in
                     (self.map, self.mask, self.root / 'map.pgm', self.root / 'mask.pgm')}
        out, count = self.build()
        params = yaml.safe_load(out.read_text())
        served = Path(params['oomwoo_keepout_mask_server']['ros__parameters']['yaml_filename'])
        self.assertNotEqual(served, self.mask)
        meta = yaml.safe_load(served.read_text())
        width, height, actual = _pgm(served.parent / meta['image'])
        self.assertEqual((width, height, count), (4, 3, 2))
        self.assertEqual(actual, bytes([255, 0, 255, 255, 255, 0] + [255] * 6))
        self.assertEqual(set(actual), {0, 255})
        self.assertEqual(meta['origin'], [-.1, -.2, 0])
        self.assertEqual(meta['resolution'], .05)
        for path, before in originals.items():
            self.assertEqual(path.read_bytes(), before)

    def test_source_raw_unknown_is_added_to_served_mask(self):
        meta = yaml.safe_load(self.map.read_text())
        meta['mode'] = 'raw'
        self.map.write_text(yaml.safe_dump(meta))
        self.pgm('map.pgm', bytes([0, 255] + [0] * 10))
        out, count = self.build()
        params = yaml.safe_load(out.read_text())
        served = Path(params['oomwoo_keepout_mask_server']['ros__parameters']['yaml_filename'])
        mask = yaml.safe_load(served.read_text())
        actual = _pgm(served.parent / mask['image'])[2]
        self.assertEqual(count, 2)
        self.assertEqual(actual[1], 0)
        self.assertEqual(actual[5], 0)

    def test_source_scale_midtones_keep_original_mask_path(self):
        meta = yaml.safe_load(self.map.read_text())
        meta['mode'] = 'scale'
        self.map.write_text(yaml.safe_dump(meta))
        self.pgm('map.pgm', bytes([128] * 12))
        out, count = self.build()
        params = yaml.safe_load(out.read_text())
        served = Path(params['oomwoo_keepout_mask_server']['ros__parameters']['yaml_filename'])
        self.assertEqual(served, self.mask)
        self.assertEqual(count, 1)

    def test_exclusive_yaml_failure_cleans_only_owned_mask_assets(self):
        self.pgm('map.pgm', bytes([128] + [255] * 11))
        existing = self.output / 'nav2_params_with_keepout.yaml'
        winner = b'another caller owns this output\n'
        existing.write_bytes(winner)
        with self.assertRaisesRegex(ValueError, 'refuse to overwrite'):
            self.build()
        self.assertEqual(existing.read_bytes(), winner)
        self.assertEqual(list(self.output.iterdir()), [existing])


if __name__ == '__main__':
    unittest.main()
