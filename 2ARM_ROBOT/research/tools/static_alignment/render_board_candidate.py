"""Render the calculated camera in a visual-only board frame, without hardware."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import cv2
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps


def main(run):
    input_run=run
    camera=json.loads((run/'camera-candidate.json').read_text())
    run=input_run/('render-'+datetime.now().strftime('%Y%m%d-%H%M%S'))
    run.mkdir(exist_ok=False)
    pose=camera['hypotheses'][camera['selected_hypothesis_index']]
    assert pose['all_corner_depth_positive']
    K=np.asarray(camera['K'])
    T_board_optical=np.asarray(pose['T_board_from_camera'])
    # The board coordinate system is the isolated render world. It is not the
    # robot/SIM world. Optical right/down/forward -> MuJoCo right/up/backward.
    optical_from_mujoco=np.diag([1.,-1.,-1.])
    R_board_mujoco=T_board_optical[:3,:3]@optical_from_mujoco
    quat=np.empty(4)
    mujoco.mju_mat2Quat(quat,R_board_mujoco.ravel())
    cfg=camera['board_geometry']['definition']
    squares=cfg['squares_xy']
    width_m,height_m=squares[0]*cfg['square_m'],squares[1]*cfg['square_m']
    board=cv2.aruco.CharucoBoard(tuple(squares),cfg['square_m'],cfg['marker_m'],cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    texture=board.generateImage((squares[0]*256,squares[1]*256),marginSize=0,borderBits=1)
    texture_path=run/'board-texture.png'
    assert not texture_path.exists()
    assert cv2.imwrite(str(texture_path),texture)
    width,height=camera['header']['color_width'],camera['header']['color_height']
    numbers=lambda values:' '.join(format(float(v),'.17g') for v in values)
    # Principal offsets are represented explicitly and the final GL frustum is
    # checked against K. No PNG rotation, camera fitting, or shared model change.
    xml=f'''<mujoco model="STATIC14_board_only_visual_candidate">
      <visual><global offwidth="{width}" offheight="{height}"/><headlight ambient="1 1 1" diffuse="0 0 0" specular="0 0 0"/></visual>
      <asset>
        <texture name="charuco" type="2d" file="{texture_path}"/>
        <material name="board_image" texture="charuco" texuniform="false" emission="1" specular="0" shininess="0"/>
        <mesh name="board_surface" vertex="0 0 0 {width_m} 0 0 {width_m} {height_m} 0 0 {height_m} 0 0 0 .001 {width_m} 0 .001 {width_m} {height_m} .001 0 {height_m} .001" face="0 2 1 0 3 2 4 5 6 4 6 7 0 1 5 0 5 4 1 2 6 1 6 5 2 3 7 2 7 6 3 0 4 3 4 7" texcoord="0 0 1 0 1 1 0 1 0 0 1 0 1 1 0 1"/>
      </asset>
      <worldbody>
        <geom name="visual_board" type="mesh" mesh="board_surface" material="board_image" contype="0" conaffinity="0" group="2"/>
        <camera name="calculated_candidate" pos="{numbers(T_board_optical[:3,3])}" quat="{numbers(quat)}" ipd="0"
          resolution="{width} {height}" sensorsize="{width} {height}" focal="{K[0,0]} {K[1,1]}" principal="{width/2-K[0,2]} {K[1,2]-height/2}"/>
      </worldbody>
    </mujoco>'''
    xml_path=run/'board-candidate.visual-only.xml'
    with xml_path.open('x') as f:f.write(xml)
    model=mujoco.MjModel.from_xml_path(str(xml_path))
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    points=np.asarray(camera['object_points_m'])
    cv_projection=np.asarray(pose['projected_corners_uv'])
    with mujoco.Renderer(model,height,width) as renderer:
        renderer.update_scene(data,camera='calculated_candidate')
        frusta=[]
        for view in renderer.scene.camera:
            n=float(view.frustum_near)
            # MuJoCo 3.3.7 render_gl3.c uses frustum_width as HALF-width.
            view.frustum_width=.5*n*width/K[0,0]
            view.frustum_center=n*(width/2-K[0,2])/K[0,0]
            view.frustum_top=n*K[1,2]/K[1,1]
            view.frustum_bottom=-n*(height-K[1,2])/K[1,1]
            left=view.frustum_center-view.frustum_width
            right=view.frustum_center+view.frustum_width
            up=np.asarray(view.up,dtype=float)
            forward=np.asarray(view.forward,dtype=float)
            right_axis=np.cross(forward,up)
            right_axis/=np.linalg.norm(right_axis)
            up/=np.linalg.norm(up);forward/=np.linalg.norm(forward)
            delta=points-np.asarray(view.pos)
            z=delta@forward
            xnear=n*(delta@right_axis)/z; ynear=n*(delta@up)/z
            projected=np.column_stack([width*(xnear-left)/(right-left),height*(view.frustum_top-ynear)/(view.frustum_top-view.frustum_bottom)])
            max_error=float(np.linalg.norm(projected-cv_projection,axis=1).max())
            assert max_error<1e-3, f'Optical/MuJoCo projection mismatch: {max_error}'
            frusta.append(dict(near=n,half_width=float(view.frustum_width),center=float(view.frustum_center),top=float(view.frustum_top),bottom=float(view.frustum_bottom),projection_max_error_vs_opencv_px=max_error))
        pixels=renderer.render()
        Image.fromarray(pixels).save(run/'candidate-sim-board.png')
    detected,ids,_,_=cv2.aruco.CharucoDetector(board).detectBoard(pixels)
    assert ids is not None and len(ids)>=6, 'Rendered board must be detectable without image rotation/reflection'
    render_check=dict(detected_corners=0 if ids is None else len(ids))
    if ids is not None and len(ids)>=6:
        obj,observed=board.matchImagePoints(detected,ids)
        rvec=np.asarray(pose['rvec']);tvec=np.asarray(pose['tvec_m'])
        projected=cv2.projectPoints(obj,rvec,tvec,K,np.zeros(5))[0].reshape(-1,2)
        error=np.linalg.norm(projected-observed.reshape(-1,2),axis=1)
        render_check.update(reprojection_rms_px=float(np.sqrt(np.mean(error**2))),max_px=float(error.max()))
        assert render_check['reprojection_rms_px']<1.5, render_check
    cfg_path=Path(camera['board_geometry']['config']['path'])
    cfg_document=json.loads(cfg_path.read_text())
    T_datum_board=np.asarray(cfg_document['T_datum_from_board'])
    result=dict(visual_only=True,hardware_access_this_work=False,render_world='OpenCV board coordinate frame; no physical robot world binding',
                board_geometry=camera['board_geometry'],pose_input=str(input_run/'camera-candidate.json'),
                T_board_camera_optical=T_board_optical.tolist(),T_board_camera_mujoco=dict(rotation=R_board_mujoco.tolist(),quaternion_wxyz=quat.tolist(),translation_m=T_board_optical[:3,3].tolist()),
                K_used=K.tolist(),intrinsics_method='sensorsize/focal/principal and explicit per-eye GL frustum from K; principal point retained',frusta=frusta,
                board_render_detection_check=render_check,
                operator_reply='1. ㅇㅇ / 2. 없어',operator_confirmed_board_datum_installation_unchanged=True,
                T_datum_camera_optical_candidate=(T_datum_board@T_board_optical).tolist(),
                datum_geometry_source=camera['board_geometry']['config'],
                T_world_camera=None,world_limitation='No independently verified datum-to-world/base rotation; candidate datum pose does not certify physical world calibration',
                xml=str(xml_path),xml_sha256=hashlib.sha256(xml_path.read_bytes()).hexdigest(),mujoco_version=mujoco.__version__)
    with (run/'camera-datum-and-render.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    sheet=Image.new('RGB',(1440,450),'#eef0f3')
    draw=ImageDraw.Draw(sheet)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16)
    items=[(input_run/'native-mono.png','REAL native mono | unchanged pixels'),
           (Path(os.environ['DAPIER_ALIGNMENT_STATIC13'])/'A-sim-os30a_UNVERIFIED.png','EXISTING SIM | nominal robot world'),
           (run/'candidate-sim-board.png','CANDIDATE SIM | board-frame visual only')]
    for index,(path,title) in enumerate(items):
        with Image.open(path) as im:tile=ImageOps.contain(im.convert('RGB'),(460,370))
        x=index*480
        draw.text((x+10,12),title,fill='#101820',font=font)
        sheet.paste(tile,(x+10+(460-tile.width)//2,48+(370-tile.height)//2))
    draw.text((10,425),'PnP under current factory P1 / D=0 hypothesis; robot-world binding and physical joint zeros remain unverified',fill='#101820',font=font)
    sheet.save(run/'native-nominal-candidate.png')
    print(json.dumps(dict(output=str(run/'native-nominal-candidate.png'),render_check=render_check,projection_max_error_px=max(v['projection_max_error_vs_opencv_px'] for v in frusta),T_datum_camera_candidate=result['T_datum_camera_optical_candidate']),indent=2))


if __name__=='__main__':
    args=argparse.ArgumentParser(description=__doc__)
    args.add_argument('--run',type=Path,required=True)
    main(args.parse_args().run)
