"""Offline configuration regressions; temporary copies only."""
import importlib.util
import json
from pathlib import Path
import shutil
import tempfile
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('validate_home', ROOT / 'tools/arm/validate_home.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class HomeConfigTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        for name in ('config/arm/home.yaml', 'config/arm/home_reference.json', 'config/arm/twist_reference.yaml', 'src/chess_robot_arm/config/home.yaml'):
            dest = self.root / name
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / name, dest)
    def tearDown(self):
        self.temp.cleanup()
    def test_valid_sources_and_provenance(self):
        self.assertEqual(module.validate(self.root)['status'], 'PASS')
    def test_drift_is_explicit(self):
        mirror = self.root / 'src/chess_robot_arm/config/home.yaml'
        mirror.write_text('chess_robot_arm: {ros__parameters: {home: {taught: false}}}')
        with self.assertRaisesRegex(ValueError, 'drift'):
            module.validate(self.root)
    def test_missing_source_metadata_fails_closed(self):
        (self.root / 'config/arm/home_reference.json').unlink()
        with self.assertRaises(OSError):
            module.validate(self.root)
    def test_comment_only_mirror_drift_also_fails(self):
        mirror = self.root / 'src/chess_robot_arm/config/home.yaml'
        mirror.write_text('# stale package header\n' + mirror.read_text())
        with self.assertRaisesRegex(ValueError, 'drift'):
            module.validate(self.root)
    def test_wrong_metadata_cannot_approve_another_home(self):
        path = self.root / 'config/arm/home_reference.json'
        data = json.loads(path.read_text()); data['joint_positions_rad'][0] += .01
        path.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, 'does not match'):
            module.validate(self.root)
    def test_interrupted_apply_fails_closed(self):
        (self.root / 'config/arm/.home_update_pending').touch()
        with self.assertRaisesRegex(ValueError, 'unfinished'):
            module.validate(self.root)
    def test_nonfinite_or_out_of_bounds_target_fails(self):
        for value in (float('nan'), 100):
            for name in ('config/arm/home.yaml', 'src/chess_robot_arm/config/home.yaml'):
                path = self.root / name
                data = yaml.safe_load(path.read_text())
                data['chess_robot_arm']['ros__parameters']['home']['joint_positions_rad'][0] = value
                path.write_text(yaml.safe_dump(data))
            with self.assertRaises(ValueError):
                module.validate(self.root)


if __name__ == '__main__':
    unittest.main()
