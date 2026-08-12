#!/usr/bin/env python3

from pathlib import Path
import tempfile
import unittest

import yaml

from tb3_map_validate import MapValidationError, validate_map


class MapValidationTests(unittest.TestCase):
    def create_map(self, directory: Path, raster: bytes = bytes(range(12))) -> Path:
        image = directory / "room.pgm"
        image.write_bytes(b"P5\n# generated test map\n4 3\n255\n" + raster)
        metadata = {
            "image": image.name,
            "mode": "trinary",
            "resolution": 0.05,
            "origin": [-1.0, -2.0, 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        }
        yaml_path = directory / "room.yaml"
        yaml_path.write_text(yaml.safe_dump(metadata), encoding="utf-8")
        return yaml_path

    def test_valid_generated_map(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            yaml_path = self.create_map(Path(directory))
            width, height, resolution, image = validate_map(yaml_path)
            self.assertEqual((width, height), (4, 3))
            self.assertEqual(resolution, 0.05)
            self.assertEqual(image.name, "room.pgm")

    def test_rejects_truncated_raster(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            yaml_path = self.create_map(Path(directory), raster=b"\x00" * 11)
            with self.assertRaisesRegex(MapValidationError, "raster size mismatch"):
                validate_map(yaml_path)

    def test_rejects_invalid_threshold_order(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            yaml_path = self.create_map(Path(directory))
            metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
            metadata["free_thresh"] = 0.8
            yaml_path.write_text(yaml.safe_dump(metadata), encoding="utf-8")
            with self.assertRaisesRegex(MapValidationError, "thresholds"):
                validate_map(yaml_path)

    def test_rejects_missing_image(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            yaml_path = self.create_map(Path(directory))
            (Path(directory) / "room.pgm").unlink()
            with self.assertRaisesRegex(MapValidationError, "does not exist"):
                validate_map(yaml_path)


if __name__ == "__main__":
    unittest.main()
