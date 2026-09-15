import contextlib
import sys
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
import mujoco
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from grasp_debug import GraspDebug, markers, run_viewer


class GraspDebugTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.debug=GraspDebug()
        cls.candidate=cls.debug.candidate(0)

    def test_explicit_preview_and_model_isolation(self):
        d=self.debug
        d.preview(self.candidate)
        np.testing.assert_array_equal(d.preview_data.qpos,d.data.qpos)
        d.preview(self.candidate,requested=True)
        self.assertFalse(np.array_equal(d.preview_data.qpos,d.data.qpos))
        self.assertEqual(d.preview_data.time,d.data.time)
        before=d.model.geom_rgba.copy()
        d.preview_model.geom_rgba[0,0]=.123
        np.testing.assert_array_equal(before,d.model.geom_rgba)
        d.preview_model.geom_rgba[:]=before
        d.restore_preview()
        np.testing.assert_array_equal(d.preview_data.qvel,d.data.qvel)
        d.assert_unchanged()

    def test_markers_use_user_scene_only(self):
        scene=mujoco.MjvScene(self.debug.preview_model,maxgeom=16)
        markers(scene,self.debug,self.candidate)
        self.assertEqual(scene.ngeom,6)
        self.assertEqual(scene.geoms[2].label,"PREGRASP")
        self.assertEqual(scene.geoms[3].label,"GRASP")
        self.debug.assert_unchanged()

    def test_viewer_keys_restore_state_and_close(self):
        import mujoco.viewer
        d=self.debug
        from types import SimpleNamespace
        class FakeViewer:
            def __init__(self,callback):
                self.callback=callback
                self.keys=iter((ord("P"),ord("1"),ord("H"),ord("Q")))
                self.user_scn=mujoco.MjvScene(d.preview_model,maxgeom=16)
                self.cam=SimpleNamespace(lookat=np.zeros(3))
                self.closed=False
                self.shown=[]
            def __enter__(self): return self
            def __exit__(self,*args): self.closed=True
            def is_running(self): return True
            def lock(self): return contextlib.nullcontext()
            def set_texts(self,*args): pass
            def sync(self):
                self.shown.append(d.preview_data.qpos.copy())
                self.callback(next(self.keys))
        handles=[]
        def launch(model,data,*,key_callback):
            self.assertIsNot(model,d.model)
            self.assertIsNot(data,d.data)
            v=FakeViewer(key_callback);handles.append(v);return v
        with (patch.object(d,"candidate",return_value=self.candidate),
              patch.object(mujoco.viewer,"launch_passive",side_effect=launch),
              patch("grasp_debug.time.sleep"),patch("builtins.print")):
            run_viewer(d)
        self.assertTrue(handles[0].closed)
        np.testing.assert_array_equal(handles[0].shown[0],d.data.qpos)
        self.assertFalse(np.array_equal(handles[0].shown[1],d.data.qpos))
        np.testing.assert_array_equal(handles[0].shown[2],d.data.qpos)
        np.testing.assert_array_equal(d.preview_data.qpos,d.data.qpos)
        d.assert_unchanged()


if __name__=="__main__":
    unittest.main()
