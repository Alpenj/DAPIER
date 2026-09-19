from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from dapier_research.basic_motion_readiness import JOINTS,inspect_snapshot
class BasicReadinessTest(unittest.TestCase):
    def test_raw_state_preserved_and_no_unverified_target(self):
        raw={n:1000+i for i,n in enumerate(JOINTS)}
        snap=dict(schema_version="dapier.dual-so101-smoke.v0.4",motion_requested_deg=0,motion_enabled=False,trusted_profile_verified=True,controller_identity_revalidated={"left":True},arms={"left":{"position_raw_tick":raw}})
        cal={n:dict(id=i+1,drive_mode=i%2,range_min=100,range_max=3900,homing_offset=123) for i,n in enumerate(JOINTS)}
        r=inspect_snapshot(snap,cal,{"rate-hz":30,"maximum-step-ticks":35,"keep-motion-settings":True})
        self.assertEqual(r["current_measured_q"],list(raw.values()))
        self.assertIsNone(r["target_q"])
        self.assertFalse(r["hardware_execution"] or r["control_authorized"])
        self.assertEqual(r["existing_profile"]["maximum-step-ticks"],35)
        self.assertEqual(raw["shoulder_pan"],1000)
        for invalid in (float("nan"),float("inf"),-1,4096):
            snap["arms"]["left"]["position_raw_tick"]["shoulder_pan"]=invalid
            with self.assertRaises(ValueError):inspect_snapshot(snap,cal,{})
    def test_wrong_role_or_motion_rejected(self):
        with self.assertRaises(ValueError):inspect_snapshot({}, {}, {})
if __name__=="__main__":unittest.main()
