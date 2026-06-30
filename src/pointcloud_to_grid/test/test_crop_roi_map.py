from __future__ import print_function

import imp
import os
import shutil
import sys
import tempfile
import unittest

import numpy as np


def load_crop_module():
    script_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', 'scripts', 'crop_roi_map.py'))
    module_name = 'crop_roi_map'
    if module_name in sys.modules:
        del sys.modules[module_name]
    return imp.load_source(module_name, script_path)


class CropRoiMapTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_crop_module()

    def test_crop_updates_map_origin_and_dimensions(self):
        temp_dir = tempfile.mkdtemp()
        try:
            input_pgm = os.path.join(temp_dir, 'input.pgm')
            input_yaml = os.path.join(temp_dir, 'input.yaml')
            output_prefix = os.path.join(temp_dir, 'roi_map')

            image = np.full((10, 12), 205, dtype=np.uint8)
            image[3:7, 2:9] = 254
            image[4:6, 5:7] = 0
            self.module.write_pgm(input_pgm, image)
            with open(input_yaml, 'w') as yaml_file:
                yaml_file.write('image: input.pgm\n')
                yaml_file.write('resolution: 0.050000\n')
                yaml_file.write('origin: [-8.500000, -31.000000, 0.000000]\n')
                yaml_file.write('negate: 0\n')
                yaml_file.write('occupied_thresh: 0.65\n')
                yaml_file.write('free_thresh: 0.196\n')

            code = self.module.main([
                '--input-pgm', input_pgm,
                '--input-yaml', input_yaml,
                '--output-prefix', output_prefix,
                '--x-min', '2',
                '--x-max', '9',
                '--y-min', '3',
                '--y-max', '8',
                '--free-expand-cells', '0',
                '--obstacle-keepout-cells', '0',
            ])

            self.assertEqual(code, 0)
            output = self.module.read_pgm(output_prefix + '.pgm')
            self.assertEqual(output.shape, (5, 7))
            with open(output_prefix + '.yaml', 'r') as yaml_file:
                yaml_text = yaml_file.read()
            self.assertIn('image: roi_map.pgm', yaml_text)
            self.assertIn('origin: [-8.400000, -30.850000, 0.000000]', yaml_text)
        finally:
            shutil.rmtree(temp_dir)


if __name__ == '__main__':
    unittest.main()
