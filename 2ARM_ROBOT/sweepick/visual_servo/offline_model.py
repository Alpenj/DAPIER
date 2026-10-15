"""Virtual pinhole features through the unchanged A pure IK and collision checker."""
import numpy as np

from .image_jacobian import estimate_jacobian

MODEL_FEATURES=dict(u='MODEL_PROJECTED_FIXED_POINT_U',v='MODEL_PROJECTED_FIXED_POINT_V')


class AModelImageLoop:
    def __init__(self,geometry,q_seed,point_model,*,viewport_hw,context,source_sha256):
        self.g=geometry;self.model=geometry.model;self.mj=geometry.mj
        self.q_seed=np.asarray(q_seed,float);self.point=np.asarray(point_model,float)
        if self.point.shape!=(3,) or not np.isfinite(self.point).all():
            raise ValueError('explicit MODEL point required')
        self.height,self.width=viewport_hw
        if self.height<=0 or self.width<=0:raise ValueError('explicit positive virtual viewport required')
        self.camera=self.model.camera('left_wrist_rgb').id
        self.site=self.model.site('left_cube_grasp').id
        self.focal=self.height/(2*np.tan(np.deg2rad(self.model.cam_fovy[self.camera])/2))
        self.context=context;self.source_sha256=source_sha256
        self.teacher=self.g.tm.HandoverTeacher(self.model)
        self.base=self.mj.MjData(self.model)
        self.g.kin.set_q12(self.model,self.base,self.q_seed)
        self.base.qpos[self.teacher.cm.block_qpos:self.teacher.cm.block_qpos+3]=self.point
        self.mj.mj_forward(self.model,self.base)
        # The caller supplies an existing recorded/model seed; measured q12 is never clipped here.
        self.g.tm.decode_command(self.q_seed,self.model)
        self.path_checks=[];self.ik_calls=0

    def observe(self,q):
        poses=self.g.fk(q);d=self.g.data
        camera_rotation=d.cam_xmat[self.camera].reshape(3,3)
        relative=(self.point-d.cam_xpos[self.camera])@camera_rotation
        depth=-relative[2]
        if not np.isfinite(relative).all() or depth<=0:
            raise ValueError('MODEL point is not in front of the model camera')
        uv=np.array([self.width/2+self.focal*relative[0]/depth,
                     self.height/2-self.focal*relative[1]/depth])
        return uv,poses['left']

    def move(self,q,delta_model_m):
        _,pose=self.observe(q);seed=self.g.tm.decode_command(q,self.model)
        self.ik_calls+=1
        goal,info=self.teacher.plan.solve(self.base.qpos,seed,'left',
            pose[:3,3]+np.asarray(delta_model_m),pose[:3,0],[pose[:3,2]],self.teacher.cfg)
        if goal is None:return None,dict(status='IK_FAILED',reason=info)
        goal[[5,11]]=seed[[5,11]]
        hit=self.teacher.plan.path_collisions(self.base.qpos,seed,goal,
            block_qpos=self.base.qpos[self.teacher.cm.block_qpos:self.teacher.cm.block_qpos+7])
        row=dict(status='CLEAR' if hit is None else 'COLLISION',
            existing_planner_margin_m=self.teacher.plan.gripper_clearance_m,
            margin_role='existing A PROVISIONAL model planning parameter; not physical approval',
            IK=info,collision=None if hit is None else self.g.tm.__name__+': '+str(hit))
        self.path_checks.append(row)
        return (self.teacher.to_canonical(goal) if hit is None else None),row

    def jacobian(self,q,basis_model,*,probe_mm):
        if not np.isfinite(probe_mm) or probe_mm<=0:raise ValueError('positive numerical finite-difference probe required')
        basis=np.asarray(basis_model,float);motion=[];pixels=[];probes=[]
        if basis.shape!=(3,2) or not np.allclose(basis.T@basis,np.eye(2)):
            raise ValueError('explicit orthonormal model translation plane required')
        for direction in basis.T:
            endpoints=[]
            for sign in (-1,1):
                solved,check=self.move(q,direction*(sign*probe_mm/1000))
                if solved is None:return dict(status='MODEL_PROBE_FAILED',jacobian=None,check=check)
                uv,tool=self.observe(solved);endpoints.append((uv,tool))
                probes.append(dict(sign=sign,IK_and_collision=check))
            pixels.append(endpoints[1][0]-endpoints[0][0])
            motion.append((endpoints[1][1][:3,3]-endpoints[0][1][:3,3])@basis*1000)
        result=estimate_jacobian(motion,pixels,feature_axes=['u','v'],motion_basis_model=basis,
            context=self.context,feature_definitions=MODEL_FEATURES,source_sha256=self.source_sha256,
            source='MODEL_ONLY_JACOBIAN',comparable_context=True)
        result.update(probe_mm=float(probe_mm),probe_role='numerical differentiation only; no hardware write or travel cap',
            probes=probes,camera='left_wrist_rgb',viewport_hw=[self.height,self.width],
            intrinsics_role='virtual square pixels from model vertical FOV; not REAL K/D',
            physical_match_verified=False)
        return result
