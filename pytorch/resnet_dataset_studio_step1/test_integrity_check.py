from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from PIL import Image

from integrity_check import inspect_dataset


class IntegrityCheckTest(unittest.TestCase):
    def create_image(self, path: Path, color: tuple[int, int, int]) -> None:
        Image.new("RGB", (224, 224), color).save(path, format="JPEG")

    def test_passes_balanced_imagefolder(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            for class_index, class_name in enumerate(("class_A", "class_B")):
                class_dir = root / class_name
                class_dir.mkdir()
                for image_index in range(2):
                    color = (class_index * 80, image_index * 60, 30)
                    self.create_image(class_dir / f"{image_index}.jpg", color)

            report = inspect_dataset(root, target=2, expected_classes=2)

            self.assertTrue(report["valid"])
            self.assertEqual(report["total"], 4)
            self.assertEqual(report["imagefolder_classes"], ["class_A", "class_B"])

    def test_fails_when_class_is_short(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            for class_name in ("class_A", "class_B"):
                class_dir = root / class_name
                class_dir.mkdir()
                self.create_image(class_dir / "0.jpg", (20, 40, 60))
            self.create_image(root / "class_A" / "1.jpg", (60, 40, 20))

            report = inspect_dataset(root, target=2, expected_classes=2)

            self.assertFalse(report["valid"])
            self.assertEqual(report["class_counts"]["class_B"], 1)


if __name__ == "__main__":
    unittest.main()
