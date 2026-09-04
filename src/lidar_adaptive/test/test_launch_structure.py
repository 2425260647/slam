#!/usr/bin/env python3
"""Static launch/config checks that do not require a running ROS master."""

import os
import unittest
import xml.etree.ElementTree as ET


class LaunchStructureTest(unittest.TestCase):
    def setUp(self):
        self.package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def test_pipeline_has_only_research_nodes(self):
        path = os.path.join(self.package_dir, "launch", "lidar_adaptive_pipeline.launch")
        root = ET.parse(path).getroot()
        names = [node.attrib.get("name") for node in root.findall("node")]
        self.assertEqual(names, ["lidar_confidence_projection", "adaptive_scan_selector"])
        text = ET.tostring(root, encoding="unicode")
        self.assertNotIn("move_base", text)
        self.assertNotIn("explorer_controller", text)

    def test_cartographer_entry_remaps_selected_scan(self):
        path = os.path.join(self.package_dir, "launch", "lidar_adaptive_cartographer.launch")
        root = ET.parse(path).getroot()
        nodes = root.findall("node")
        cartographer = [n for n in nodes if n.attrib.get("type") == "cartographer_node"]
        self.assertEqual(len(cartographer), 1)
        remaps = {(r.attrib.get("from"), r.attrib.get("to"))
                  for r in cartographer[0].findall("remap")}
        self.assertIn(("scan", "/scan_selected"), remaps)


if __name__ == "__main__":
    unittest.main()
