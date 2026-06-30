from __future__ import print_function

import imp
import os
import shutil
import sys
import tempfile
import unittest

import numpy as np


def load_wall_module():
    script_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', 'scripts', 'manual_wall_connect_map.py'))
    module_name = 'manual_wall_connect_map'
    if module_name in sys.modules:
        del sys.modules[module_name]
    return imp.load_source(module_name, script_path)


class ManualWallConnectMapTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_wall_module()

    def test_draw_line_connects_wall_gap(self):
        image = np.full((8, 12), 254, dtype=np.uint8)
        self.module.draw_line(image, 2, 4, 9, 4, value=0, thickness=1)

        self.assertTrue(np.all(image[4, 2:10] == 0))
        self.assertEqual(int(image[3, 5]), 254)

    def test_main_writes_connected_map_and_yaml(self):
        temp_dir = tempfile.mkdtemp()
        try:
            input_pgm = os.path.join(temp_dir, 'input.pgm')
            input_yaml = os.path.join(temp_dir, 'input.yaml')
            output_prefix = os.path.join(temp_dir, 'connected')
            image = np.full((12, 16), 254, dtype=np.uint8)
            self.module.write_pgm(input_pgm, image)
            with open(input_yaml, 'w') as yaml_file:
                yaml_file.write('image: input.pgm\n')
                yaml_file.write('resolution: 0.100000\n')
                yaml_file.write('origin: [0.000000, 0.000000, 0.000000]\n')
                yaml_file.write('negate: 0\n')

            self.module.main([
                '--input-pgm', input_pgm,
                '--input-yaml', input_yaml,
                '--output-prefix', output_prefix,
                '--preset', 'none',
                '--segment', '1,6,8,6',
                '__name:=manual_wall_connect_map',
            ])

            output = self.module.read_pgm(output_prefix + '.pgm')
            self.assertTrue(np.all(output[6, 1:9] == 0))
            with open(output_prefix + '.yaml', 'r') as yaml_file:
                yaml_text = yaml_file.read()
            self.assertIn('image: connected.pgm', yaml_text)
            self.assertIn('resolution: 0.100000', yaml_text)
        finally:
            shutil.rmtree(temp_dir)

    def test_default_my_map_corners_are_l_shaped(self):
        image = np.full((115, 137), 254, dtype=np.uint8)
        for segment in self.module.default_my_map_cropped_segments():
            self.module.draw_line(image, segment[0], segment[1],
                                  segment[2], segment[3],
                                  value=0, thickness=2)

        self.assertTrue(np.all(image[45:48, 67:71] == 254))
        self.assertTrue(np.all(image[48:52, 64:67] == 254))
        self.assertTrue(np.all(image[77:81, 66:71] == 254))
        self.assertTrue(np.all(image[74:78, 63:66] == 254))

    def test_selected_noise_rectangles_clear_only_local_patches(self):
        temp_dir = tempfile.mkdtemp()
        try:
            input_pgm = os.path.join(temp_dir, 'input.pgm')
            output_prefix = os.path.join(temp_dir, 'cleaned')
            image = np.full((115, 137), 254, dtype=np.uint8)
            for x1, y1, x2, y2 in self.module.default_my_map_cropped_noise_rects():
                image[y1:y2 + 1, x1:x2 + 1] = 0
            image[79, 40] = 0
            image[46, 20] = 0
            self.module.write_pgm(input_pgm, image)

            self.module.main([
                '--input-pgm', input_pgm,
                '--output-prefix', output_prefix,
                '--preset', 'my_map_cropped',
                '--clear-selected-noise',
            ])

            output = self.module.read_pgm(output_prefix + '.pgm')
            for x1, y1, x2, y2 in self.module.default_my_map_cropped_noise_rects():
                self.assertTrue(np.all(output[y1:y2 + 1, x1:x2 + 1] == 254))
            self.assertEqual(int(output[79, 40]), 0)
            self.assertEqual(int(output[46, 20]), 0)
        finally:
            shutil.rmtree(temp_dir)

    def test_selected_corner_wall_and_isolated_points_are_fixed(self):
        temp_dir = tempfile.mkdtemp()
        try:
            input_pgm = os.path.join(temp_dir, 'input.pgm')
            output_prefix = os.path.join(temp_dir, 'fixed')
            image = np.full((115, 137), 254, dtype=np.uint8)
            self.module.write_pgm(input_pgm, image)

            self.module.main([
                '--input-pgm', input_pgm,
                '--output-prefix', output_prefix,
                '--preset', 'my_map_cropped',
                '--clear-selected-noise',
            ])

            output = self.module.read_pgm(output_prefix + '.pgm')
            self.assertTrue(np.all(output[42:45, 62:66] == 0))
            self.assertTrue(np.all(output[45:48, 62:66] == 0))
            self.assertTrue(np.all(output[45:48, 66:68] == 254))
            self.assertTrue(np.all(output[75:78, 58:65] == 254))
            self.assertTrue(np.all(output[78:81, 58:63] == 0))
            self.assertTrue(np.all(output[91:97, 66:69] == 254))
            self.assertTrue(np.all(output[80:97, 64:65] == 0))
            self.assertTrue(np.any(output[80:85, 63:66] == 0))
            self.assertTrue(np.all(output[35:41, 64:67] == 0))
            self.assertTrue(np.all(output[35:41, 67:70] == 254))
            self.assertTrue(np.all(output[75:78, 52:58] == 254))
            self.assertTrue(np.all(output[82:96, 66:71] == 254))
            self.assertTrue(np.all(output[82:96, 63:65] == 0))
        finally:
            shutil.rmtree(temp_dir)


if __name__ == '__main__':
    unittest.main()
