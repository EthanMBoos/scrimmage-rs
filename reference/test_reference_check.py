import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import reference_check as check


class ReferenceCheckTests(unittest.TestCase):
    def test_check_directories_are_numbered_and_never_reused(self):
        with tempfile.TemporaryDirectory() as directory:
            runs = Path(directory).resolve() / "runs"
            self.assertEqual(check.allocate_output(None, runs), runs / "reference-check000")
            (runs / "reference-check007").mkdir()
            self.assertEqual(check.allocate_output(None, runs), runs / "reference-check008")
            explicit = Path(directory).resolve() / "nested/named"
            self.assertEqual(check.allocate_output(explicit, runs), explicit)
            with self.assertRaises(FileExistsError):
                check.allocate_output(explicit, runs)

    def test_headless_overrides_preserve_physics_seed_and_multiplier_typo(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            mission = directory / "mission.xml"
            mission.write_text('''<runscript>
              <run dt="0.1" end="200" motion_multipler="5" enable_gui="true" />
              <seed>12345</seed><multi_threaded num_threads="8">true</multi_threaded>
              <entity><x>23</x><autonomy speed="27">Straight</autonomy></entity>
            </runscript>''')
            original = mission.read_bytes()
            metadata = check.prepare_mission(mission, directory / "cpp")
            root = ET.parse(directory / "cpp/effective.xml").getroot()
            self.assertEqual(root.find("run").get("enable_gui"), "false")
            self.assertEqual(root.find("run").get("network_gui"), "false")
            self.assertEqual(root.find("run").get("motion_multipler"), "5")
            self.assertIsNone(root.find("run").get("motion_multiplier"))
            self.assertEqual(root.findtext("seed"), "12345")
            self.assertEqual(root.findtext("entity/x"), "23")
            self.assertEqual(root.find("entity/autonomy").get("speed"), "27")
            self.assertEqual(root.findtext("multi_threaded"), "false")
            self.assertEqual(root.findtext("log_dir"), "/run/logs")
            self.assertEqual(root.findtext("create_latest_dir"), "false")
            self.assertEqual(mission.read_bytes(), original)
            self.assertEqual(metadata["input_sha256"], check.sha256(mission))

    def test_includes_fail_explicitly(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            mission = directory / "mission.xml"
            mission.write_text('<runscript xmlns:xi="http://www.w3.org/2001/XInclude">'
                               '<run/><xi:include href="other.xml"/></runscript>')
            with self.assertRaisesRegex(ValueError, "XIncludes unsupported"):
                check.prepare_mission(mission, directory / "cpp")

    def test_missing_output_is_not_a_successful_comparison(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            with self.assertRaises(FileNotFoundError):
                check.same_outputs(directory, directory)

    def test_event_difference_is_not_hidden_by_matching_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            left, right = Path(directory) / "left", Path(directory) / "right"
            left.mkdir()
            right.mkdir()
            for name in check.OUTPUT_FILES:
                (left / name).write_text("same")
                (right / name).write_text("same")
            (right / "events.json").write_text("different")
            self.assertEqual(check.same_outputs(left, right), {
                "frames.bin": True, "events.json": False, "summary.csv": True,
            })

    def test_build_failure_writes_a_failed_report(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            mission = directory / "mission.xml"
            mission.write_text("<runscript><run/></runscript>")
            output = directory / "check"
            with patch.object(check, "capture", return_value="docker-test"), \
                 patch.object(check, "build_reference", side_effect=RuntimeError("build failed")):
                code = check.main(["--mission", str(mission), "--output", str(output)])
            self.assertEqual(code, 1)
            report = json.loads((output / "report.json").read_text())
            self.assertFalse(report["passed"])
            self.assertEqual(report["error"], "build failed")
            self.assertFalse(report["direct_cpp_event_stream_compared"])

    def test_failed_cpp_comparison_retains_all_rust_checks(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary) / "case"
            mission = Path(temporary) / "mission.xml"
            mission.write_text("<runscript><run/></runscript>")

            def fake_run(command, log, **kwargs):
                if command[0] == "docker":
                    target = directory / "cpp/logs/reference"
                    target.mkdir(parents=True)
                    (target / "frames.bin").write_bytes(b"cpp frames")
                    (target / "summary.csv").write_text("cpp summary")
                elif command[1] == "run":
                    target = Path(command[command.index("--output") + 1])
                    target.mkdir()
                    for name in check.OUTPUT_FILES:
                        (target / name).write_text("identical Rust output")
                    if "--headless" in command:
                        (target / "recording.rrd").write_bytes(b"recording fixture")
                else:
                    self.assertEqual(command[:2], [check.sys.executable,
                                                   check.ROOT / "reference/compare.py"])
                    report = Path(command[command.index("--report") + 1])
                    check.write_json(report, {"passed": False, "mismatch_count": 3})
                    return 1
                return 0

            with patch.object(check, "run_logged", side_effect=fake_run), \
                 patch.object(check.subprocess, "run"):
                result = check.check_mission(SimpleNamespace(timeout=1), mission,
                                             directory, "image-id", Path("scrimmage"))
            self.assertFalse(result["passed"])
            self.assertEqual(result["comparison"]["mismatch_count"], 3)
            self.assertTrue(all(result["recording_equality"].values()))
            self.assertTrue(all(all(values.values()) for values in result["worker_equality"].values()))
            self.assertEqual(json.loads((directory / "result.json").read_text()), result)

    def test_failed_case_does_not_skip_the_rest_of_the_matrix(self):
        with tempfile.TemporaryDirectory() as temporary:
            mission = Path(temporary) / "mission.xml"
            mission.write_text("<runscript><run/></runscript>")
            output = Path(temporary) / "check"
            cases = [{"passed": False}, {"passed": True}]
            with patch.object(check, "capture", return_value="docker-test"), \
                 patch.object(check, "build_reference", return_value={"image": "test-image"}), \
                 patch.object(check, "build_rust", return_value=(Path("scrimmage"), {})), \
                 patch.object(check, "check_mission", side_effect=cases) as run_case:
                code = check.main(["--mission", str(mission), "--mission", str(mission),
                                   "--output", str(output)])
            self.assertEqual(code, 1)
            self.assertEqual(run_case.call_count, 2)
            self.assertEqual(json.loads((output / "report.json").read_text())["cases"], cases)


if __name__ == "__main__":
    unittest.main()
