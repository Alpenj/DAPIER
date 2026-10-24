import mujoco as mj
import mujoco.viewer
import numpy as np
import time
import os 
from mujoco.glfw import glfw

# MJCF 파일 로드

#model = mujoco.MjModel.from_xml_path("./101_2wheel_mujoco_basic.xml")
#data = mujoco.MjData(model)

xml_name = "2wheel_robot.xml"


dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname+'/'+xml_name)

print(dirname)
print(abspath)

model = mj.MjModel.from_xml_path(abspath)
data = mj.MjData(model) 

with mujoco.viewer.launch_passive(model, data) as viewer:
    print("MuJoCo 시뮬레이터 실행 중... ESC를 눌러 종료하세요.")
    start = time.time()
    while viewer.is_running():
        step_start = time.time()

        # 단순한 제어: 좌회전 → 우회전 → 정지 반복
        t = time.time() - start
        print(t)
        if t < 10:
            data.ctrl[0] = 10  # left wheel
            data.ctrl[1] = 10  # right wheel
        elif t < 20:
            data.ctrl[0] = 10
            data.ctrl[1] = -10
        else:
            data.ctrl[:] = 0
        
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(max(0, 0.01 - (time.time() - step_start)))
