"""RIGHT_X_FINITE_METROLOGY: the operator-approved finite right ARM5 X measurement (T reference, F fit, V independent check).

Pure contract: no device, model or numpy import. The magnitudes and the axis are operator inputs of one approval; this
module adds no distance, speed or tolerance limit. Every move is still planned from the latest actual by the existing A
IK/collision/native path (sweepick_full_chain.Owner.plan_observation_candidate) and executed by the existing segment()."""
import math

SCOPE = 'RIGHT_X_FINITE_METROLOGY'
MOVE_SCOPE = 'RIGHT_X_FINITE_METROLOGY_MOVE'
T_POSES = ('AFTER_RIGHT_PREGRASP', 'AFTER_ADOPTED_VIEW')
ROLES = ('T', 'F', 'V')
CONFIG_KEYS = {'t_pose', 'x_basis_tool', 'x_basis_source', 'fit_displacement_m', 'verify_displacement_m', 'verify_from'}

# Every motor/torque write of an approved session, in order, as the existing chain performs it (normal end). The measurement
# adds only the right ARM5 F/V moves and their reversal; nothing else is new.
DECLARED_WRITES = (
    'LEFT ARM+GRIPPER PICK: pregrasp, align, descend_a, descend (Goal_Position, Torque_Enable)',
    'LEFT GRIPPER CLOSE on the block (Goal_Position, Torque_Enable)',
    'LEFT ARM LIFT (Goal_Position)',
    'RIGHT ARM5 CARRY_RIGHT (Goal_Position, Torque_Enable)',
    'LEFT ARM CARRY_LEFT (Goal_Position)',
    'RIGHT ARM5 RIGHT_APPROACH (Goal_Position)',
    'RIGHT ARM5 adopted observation view, only when t_pose=AFTER_ADOPTED_VIEW, by its own operator decision (Goal_Position)',
    'RIGHT ARM5 X move F, by its own operator decision (Goal_Position)',
    'RIGHT ARM5 X move V, by its own operator decision (Goal_Position)',
    'RIGHT ARM5 normal reverse of every executed X/view move, RIGHT_BACK_TO_CARRY (Goal_Position)',
    'RIGHT_RETURN_INITIAL with right Torque OFF (Goal_Position, Torque_Enable)',
    'LEFT_BACK_TO_LIFT (Goal_Position)',
    'LEFT existing put-back on the desk: lower, open (left gripper opens on the desk), retreat, home with left Torque OFF (Goal_Position, Torque_Enable)',
)
EXCLUDED = ('RIGHT gripper command', 'RIGHT INSERT', 'RIGHT CLOSE', 'LEFT RELEASE / load transfer to the right hand',
            'image-based correction command', 'servo setting/EEPROM write', 'automatic retry, reconnect or recovery')


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def config_errors(cfg):
    """Malformed or incomplete operator input. An empty list means well-formed, not physically safe."""
    if not isinstance(cfg, dict):
        return ['right_x_metrology must be an object']
    errors = []
    if set(cfg) != CONFIG_KEYS:
        errors.append(f'right_x_metrology fields must be exactly {sorted(CONFIG_KEYS)}')
    if cfg.get('t_pose') not in T_POSES:
        errors.append(f't_pose must be one of {T_POSES}')
    basis = cfg.get('x_basis_tool')
    if not (isinstance(basis, list) and len(basis) == 3 and all(_number(v) for v in basis)):
        errors.append('x_basis_tool must be three finite numbers in the CURRENT_TOOL frame of right_cube_grasp')
    elif not math.isclose(math.sqrt(sum(v * v for v in basis)), 1.0, rel_tol=0, abs_tol=1e-9):
        errors.append('x_basis_tool must be a unit vector (direction only; the displacement carries the size)')
    if not isinstance(cfg.get('x_basis_source'), str) or not cfg.get('x_basis_source').strip():
        errors.append('x_basis_source must state the evidence for the axis (e.g. "A model tool +X, physical direction unconfirmed")')
    for key in ('fit_displacement_m', 'verify_displacement_m'):
        if not _number(cfg.get(key)) or cfg.get(key) == 0:
            errors.append(f'{key} must be a finite non-zero signed metre value chosen by the operator')
    if cfg.get('verify_from') != 'F':
        errors.append('verify_from must be "F": V is one further move from the measured F endpoint, kept out of the fit')
    return errors


def x_relative_tool(basis, displacement_m):
    """Rotation-free CURRENT_TOOL transform for one move (list of lists, no numpy)."""
    t = [float(b) * float(displacement_m) for b in basis]
    return [[1.0, 0.0, 0.0, t[0]], [0.0, 1.0, 0.0, t[1]], [0.0, 0.0, 1.0, t[2]], [0.0, 0.0, 0.0, 1.0]]


def move_candidate(role, cfg):
    displacement = cfg['fit_displacement_m'] if role == 'F' else cfg['verify_displacement_m']
    return dict(id=f'RIGHT_X_METROLOGY_{role}', source_kind='SENSOR_SIM_CANDIDATE', role=f'RIGHT_X_{role}_MOVE_PROPOSAL_NOT_MEASUREMENT',
                targets=[dict(side='right', reference_frame='CURRENT_TOOL', reference_from_target_tool=x_relative_tool(cfg['x_basis_tool'], displacement))],
                expected_observed_components=['right wrist RGB at this endpoint', 'top and left wrist context', 'both arms actual ticks'],
                observation_policy='FINAL_TARGET_ONLY', commanded_tool_displacement_m=float(displacement),
                x_basis_tool=list(cfg['x_basis_tool']), x_basis_source=cfg['x_basis_source'], physical_success=False)


def approval_errors(approval, session_out):
    """The RIGHT_X approval must be its own scope, with the full write list acknowledged and no START pre-approval."""
    errors = []
    if approval.get('scope') != SCOPE or approval.get('execution_scope') != SCOPE:
        errors.append(f'approval scope/execution_scope must both be {SCOPE}')
    if approval.get('session_out') != str(session_out):
        errors.append('approval session_out differs from this session')
    if list(approval.get('declared_motor_writes') or []) != list(DECLARED_WRITES):
        errors.append('declared_motor_writes must list exactly the writes of this scope (right_x_metrology.DECLARED_WRITES)')
    if list(approval.get('excluded') or []) != list(EXCLUDED):
        errors.append('excluded must list exactly right_x_metrology.EXCLUDED')
    if approval.get('right_gripper_commands_permitted') is not False or approval.get('insertion_permitted') is not False \
            or approval.get('close_permitted') is not False or approval.get('left_release_permitted') is not False:
        errors.append('right gripper, insertion, close and left release must all be explicitly false')
    if approval.get('observation_view_preapproval') is not None:
        errors.append('START pre-approval is not part of RIGHT_X_FINITE_METROLOGY; every move is an explicit operator decision')
    errors += config_errors(approval.get('right_x_metrology'))
    return errors
