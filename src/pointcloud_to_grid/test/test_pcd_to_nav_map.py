from __future__ import print_function

import imp
import os
import re
import shutil
import sys
import tempfile
import unittest


def load_converter_module():
    script_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', 'scripts', 'pcd_to_nav_map.py'))
    module_name = 'pcd_to_nav_map'
    if module_name in sys.modules:
        del sys.modules[module_name]
    return imp.load_source(module_name, script_path)


class PcdToNavMapTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_converter_module()
        cls.pcd_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'output_downsampled.pcd'))
        cls.points, cls.header = cls.module.load_pcd(cls.pcd_path)

    def test_load_real_pcd(self):
        self.assertEqual(self.header['points'], 39707)
        self.assertEqual(self.points.shape[0], 39707)
        self.assertGreaterEqual(self.points.shape[1], 3)
        self.assertLess(float(self.points[:, 2].min()), -1.0)
        self.assertGreater(float(self.points[:, 2].max()), 0.5)
        self.assertLess(float(self.points[:, 2].max()), 1.0)

    def test_converter_uses_python27_compatible_style(self):
        script_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', 'scripts', 'pcd_to_nav_map.py'))
        with open(script_path, 'r') as script_file:
            text = script_file.read()

        self.assertIn('#!/usr/bin/env python2', text.splitlines()[0])
        forbidden_tokens = [
            'from __future__ import annotations',
            'from dataclasses import',
            'from pathlib import',
            'from typing import',
            'import importlib.util',
            'TemporaryDirectory',
            'encoding=',
            'exist_ok=',
            '.tobytes(',
        ]
        for token in forbidden_tokens:
            self.assertNotIn(token, text)

        self.assertIsNone(re.search(r'def\s+\w+\([^)]*:\s*[^)]*\)\s*->', text))
        self.assertIsNone(re.search(r'def\s+\w+\([^)]*\)\s*->', text))

    def test_build_map_has_free_and_occupied_cells(self):
        params = self.module.MapParams()
        grid = self.module.build_occupancy_grid(self.points, params)

        occupied = int((grid.data == 100).sum())
        free = int((grid.data == 0).sum())

        self.assertGreater(grid.width, 10)
        self.assertGreater(grid.height, 10)
        self.assertGreater(occupied, 100)
        self.assertGreater(free, 100)

    def test_arg_parser_ignores_roslaunch_private_arguments(self):
        args = self.module._parse_args([
            '--pcd', self.pcd_path,
            '--output', 'output_map',
            '__name:=pcd_to_nav_map',
            '__log:=/tmp/pcd_to_nav_map.log',
        ])

        self.assertEqual(args.pcd, self.pcd_path)
        self.assertEqual(args.output, 'output_map')

    def test_save_map_writes_pgm_and_yaml(self):
        params = self.module.MapParams()
        grid = self.module.build_occupancy_grid(self.points, params)

        temp_dir = tempfile.mkdtemp()
        try:
            image_path, yaml_path = self.module.save_map(
                grid, os.path.join(temp_dir, 'corridor_map'))

            self.assertTrue(os.path.exists(image_path))
            self.assertTrue(os.path.exists(yaml_path))
            with open(image_path, 'rb') as image_file:
                self.assertEqual(image_file.read(2), b'P5')
            with open(yaml_path, 'r') as yaml_file:
                text = yaml_file.read()
            self.assertIn('image: corridor_map.pgm', text)
            self.assertIn('resolution: 0.050000', text)
        finally:
            shutil.rmtree(temp_dir)


if __name__ == '__main__':
    unittest.main()
