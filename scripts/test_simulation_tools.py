#!/usr/bin/env python3
"""Pure regression checks for scene transitions and replay accounting; ROS is not required."""

import csv
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/embertrace_core/python"))
from embertrace_tools.fire_scene import state_model
from mission_clearance import TreeClearance


class ClearanceTests(unittest.TestCase):
    def test_turning_a_square_corner_can_touch_a_trunk(self):
        model = TreeClearance([('trunk', .69, .69, .28)],
                              [[.5, .5], [.5, -.5], [-.5, -.5], [-.5, .5]])
        self.assertLess(model.closest(0, 0, 0)[0], 0)
        self.assertGreater(model.closest(0, 0, 3.141592653589793/4)[0], .19)
        self.assertLess(model.closest(.69, .69, 0)[0], -.28)

    def test_both_forests_supply_their_actual_trunks(self):
        package = ROOT/'src/embertrace_sim'
        for world, count in [('embertrace_forest.sdf', 20), ('embertrace_forest_dense.sdf', 60)]:
            model = TreeClearance.from_files(package/'worlds'/world,
                                            package/'config/nav2_params_husky1.yaml')
            self.assertEqual(len(model.trees), count)
            self.assertGreater(model.closest(-14, -14, 0)[0], 0)

    def test_enclosing_circle_protects_every_heading(self):
        model = TreeClearance([('trunk', .69, .69, .28)], [], .8)
        first = model.closest(0, 0, 0)[0]
        self.assertLess(first, 0)
        self.assertAlmostEqual(first, model.closest(0, 0, 1.2)[0])


class SceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        models = ROOT / "src/embertrace_sim/models"
        cls.tree = (models / "ember_tree/model.sdf").read_text()
        cls.fire = (models / "fire_source/model.sdf").read_text()

    def model(self, state):
        return ET.fromstring(state_model(self.tree, self.fire, state)).find("model")

    def test_healthy_preserves_tree_geometry(self):
        original = ET.fromstring(self.tree).find("model")
        healthy = self.model("healthy")
        for tag in ("collision", "geometry"):
            self.assertEqual(
                len(original.findall(".//" + tag)), len(healthy.findall(".//" + tag))
            )
        self.assertNotIn(
            "flame_outer",
            [visual.get("name") for visual in healthy.findall(".//visual")],
        )

    def test_burning_adds_visible_fire_and_heat(self):
        model = self.model("burning")
        self.assertIn(
            "flame_outer", [visual.get("name") for visual in model.findall(".//visual")]
        )
        self.assertTrue(
            all(
                float(value.text) > 500
                for value in model.findall(".//plugin/temperature")
            )
        )
        self.assertEqual(
            len(set(link.get("name") for link in model.findall("link"))),
            len(model.findall("link")),
        )

    def test_burnt_is_dark_and_cooler(self):
        model = self.model("burnt")
        self.assertTrue(
            all(float(v.text) < 330 for v in model.findall(".//plugin/temperature"))
        )
        self.assertTrue(
            all(
                v.text == "0.06 0.05 0.04 1"
                for v in model.findall(".//material/diffuse")
            )
        )

    def test_invalid_state_rejected(self):
        with self.assertRaises(ValueError):
            self.model("unknown")

    def test_named_pose_preserved(self):
        model = ET.fromstring(
            state_model(self.tree, self.fire, "healthy", "test_tree", "4 5 0 0 0 0")
        ).find("model")
        self.assertEqual(model.get("name"), "test_tree")
        self.assertEqual(model.findtext("pose"), "4 5 0 0 0 0")


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="embertrace_replay_test_")
        self.path = Path(self.directory.name) / "trial.csv"
        self.executable = Path(
            os.environ.get(
                "EMBERTRACE_EVALUATOR",
                str(ROOT / "log/perception-validation/evaluate_perception"),
            )
        )
        if not self.executable.is_file():
            self.skipTest(
                "Run scripts/validate_perception.sh to compile evaluator first"
            )

    def tearDown(self):
        self.directory.cleanup()

    def write(self, rows):
        header = "trial,scenario,now_s,stamp_s,smoke_ppm,temperature_c,wind_speed_mps,wind_direction_rad,visual_stamp_s,visual_confidence,visual_valid,environment_available,fire_present"
        with self.path.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(header.split(","))
            writer.writerows(rows)

    def test_dropout_counts_unavailable_positive_frames(self):
        rows = [
            ["one", "dropout", 1 + i * 0.2, 1 + i * 0.2, 1, 22, 2.5, 0, 0, 0, 0, 0, 1]
            for i in range(5)
        ]
        self.write(rows)
        result = subprocess.run(
            [str(self.executable), str(self.path)],
            text=True,
            capture_output=True,
            check=True,
        )
        from io import StringIO

        metrics = next(csv.DictReader(StringIO(result.stdout)))
        self.assertEqual(int(metrics["invalid"]), 5)
        self.assertEqual(int(metrics["fn_including_unavailable"]), 5)
        self.assertEqual(int(metrics["missed_events"]), 1)
        self.assertEqual(float(metrics["end_to_end_detection_rate"]), 0)

    def test_clock_rewind_requires_new_trial_id(self):
        self.write(
            [
                ["one", "clear", 2, 2, 1, 22, 2.5, 0, 0, 0, 0, 1, 0],
                ["one", "clear", 1, 1, 1, 22, 2.5, 0, 0, 0, 0, 1, 0],
            ]
        )
        result = subprocess.run(
            [str(self.executable), str(self.path)], capture_output=True, text=True
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("new trial ID", result.stderr)

    def test_invalid_boolean_rejected(self):
        self.write([["one", "clear", 1, 1, 1, 22, 2.5, 0, 0, 0, "false", 1, 0]])
        self.assertNotEqual(
            subprocess.run(
                [str(self.executable), str(self.path)], capture_output=True
            ).returncode,
            0,
        )


if __name__ == "__main__":
    unittest.main()
