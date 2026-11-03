import mujoco
import os

# 현재 파일이 있는 디렉토리로 작업 디렉토리 강제 변경
os.chdir(os.path.dirname(os.path.abspath(__file__)))

urdf_filename = "so101_new_calib.urdf"

try:
    # 절대 경로를 유닉스 스타일(forward slash)로 변환하여 전달
    abs_path = os.path.abspath(urdf_filename).replace(os.sep, '/')
    model = mujoco.MjModel.from_xml_path(abs_path)
    
    mujoco.mj_saveLastXML("model.xml", model)
    print("Success! model.xml 생성 완료")
except Exception as e:
    print(f"Error: {e}")