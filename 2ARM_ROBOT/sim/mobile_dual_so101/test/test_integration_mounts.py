import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from check_integration_mounts import main


class IntegrationMountTest(unittest.TestCase):
    def test_cad_mount_alignment_and_preserved_layout(self):
        main()
