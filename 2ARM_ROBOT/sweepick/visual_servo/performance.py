"""CPU timings and counterfactual scalar-response models; no physical time/accuracy claims."""
import time

import cv2
import numpy as np

from .image_jacobian import solve_correction


def benchmark(function,*,repeats):
    if not isinstance(repeats,int) or isinstance(repeats,bool) or repeats<1:
        raise ValueError('positive CPU benchmark repeat count required')
    started=time.perf_counter_ns();function();cold=(time.perf_counter_ns()-started)/1e6
    samples=[]
    for _ in range(repeats):
        started=time.perf_counter_ns();function();samples.append((time.perf_counter_ns()-started)/1e6)
    return dict(clock='perf_counter_ns',unit='ms',first_call_ms=cold,repeats=repeats,
        median_ms=float(np.median(samples)),p95_ms=float(np.percentile(samples,95)),
        min_ms=min(samples),max_ms=max(samples),samples_ms=samples,
        scope='CPU function call only; no camera, actuator, settling or physical cycle time')


def roi_object_feature(image,functions):
    """Global component selection, then the same A extractor on its morphology-supported ROI."""
    _,_,stats=functions._wrist_red_components(image)
    if len(stats)<2:return functions.wrist_object(image),dict(status='FULL_FRAME_NO_COMPONENT')
    selected=1+int(np.argmax(stats[1:,cv2.CC_STAT_AREA]))
    x,y,width,height=map(int,stats[selected,:4])
    # Two 5x5 opening passes reach four source pixels: algorithm support, not a physical margin.
    halo=2*(5//2)
    x0,y0=max(0,x-halo),max(0,y-halo)
    x1,y1=min(image.shape[1],x+width+halo),min(image.shape[0],y+height+halo)
    result=functions.wrist_object(image[y0:y1,x0:x1])
    metadata=dict(status='GLOBAL_COMPONENT_THEN_ROI',roi_xyxy=[x0,y0,x1,y1],
        pixel_fraction=(x1-x0)*(y1-y0)/(image.shape[0]*image.shape[1]),
        global_component_scan_retained=True,physical_acceptance_added=False)
    if not result.get('visible'):return functions.wrist_object(image),dict(metadata,status='FULL_FRAME_FALLBACK')
    if result.get('touches_border') and (x0>0 or x1<image.shape[1]):
        return functions.wrist_object(image),dict(metadata,status='FULL_FRAME_FALLBACK')
    result=dict(result,centre_px=result['centre_px']+x0,row=result['row']+y0,
        edges_px=[value+x0 for value in result['edges_px']])
    result['touches_border']=bool(result['edges_px'][0]<=1 or result['edges_px'][1]>=image.shape[1]-2)
    return result,metadata


def candidate_correction(error_px,prior_gain,local_gain,initial_error_px,method):
    values=np.asarray([error_px,prior_gain,local_gain,initial_error_px],float)
    if not np.isfinite(values).all() or prior_gain==0 or local_gain==0:
        raise ValueError('finite nonzero scalar responses required')
    if method in ('A','E'):return -error_px/prior_gain
    if method=='B':return -error_px/local_gain
    if method=='C':
        return solve_correction([[prior_gain]],[error_px],damping=abs(prior_gain))['coefficients_mm'][0]
    if method=='D':
        weight=0.0 if initial_error_px==0 else abs(error_px)/abs(initial_error_px)
        return -error_px/(prior_gain*(1+weight))
    raise ValueError('unknown offline comparison method')


def scalar_response_comparison(pairs,*,test_target_px=.2,iteration_budget=20):
    """Compare five candidates on an explicitly fitted command-to-error response model."""
    if test_target_px<=0 or not np.isfinite(test_target_px) or iteration_budget<1:
        raise ValueError('positive numerical test target and iteration budget required')
    comparisons=[]
    for index,pair in enumerate(pairs):
        error0=pair['before_image_error_px']
        response=pair['normalized_error_response_px_per_mm']
        command_to_FK=pair['actual_model_FK_projected_tool_move_mm']/pair['commanded_tool_move_mm']
        command_response=response*command_to_FK
        prior=.8/pair['scale_mm_per_px'][0]
        other=[p['normalized_error_response_px_per_mm'] for i,p in enumerate(pairs) if i!=index]
        if not other:raise ValueError('a separate stored response is needed for the local-gain candidate')
        local=float(np.mean(other))
        for method in ('A','B','C','D','E'):
            errors=[error0];commands=[];overshoot=0
            for _ in range(iteration_budget):
                if abs(errors[-1])<=test_target_px:break
                move=candidate_correction(errors[-1],prior,local,error0,method)
                updated=errors[-1]+command_response*move
                overshoot+=int(errors[-1]*updated<0);commands.append(move);errors.append(updated)
                if not np.isfinite(updated):break
            comparisons.append(dict(pair_id=pair['id'],candidate=method,
                status='FITTED_SCALAR_MODEL_TARGET_REACHED' if abs(errors[-1])<=test_target_px else 'NUMERICAL_TEST_BUDGET_ENDED',
                predicted_corrections=len(commands),image_error_px=errors,commands_mm=commands,
                final_absolute_error_px=abs(errors[-1]),first_step_absolute_error_reduction_fraction=
                    None if error0==0 else 1-abs(errors[1] if len(errors)>1 else error0)/abs(error0),
                sign_crossings=overshoot,oscillatory_trace=overshoot>1,
                controller_prior_px_per_mm=prior,local_gain_from_other_contexts_px_per_mm=local,
                response_per_actual_FK_mm=response,actual_FK_per_commanded_mm=command_to_FK,
                fitted_response_per_commanded_mm=command_response,
                independent_REAL_validation=False,constant_context_validated=False,
                IK_or_collision_certified=False,physical_time_saved_s=None,REAL_success=False))
    return dict(status='OFFLINE_COUNTERFACTUAL_ONLY',comparisons=comparisons,
        response_model='delta error = measured local px/FK-mm * recorded FK-mm/command-mm * hypothetical command-mm',
        fit_scope='same-pair post-outcome response defines the plant; local controller gain uses other contexts',
        limitations=['not an independent prediction or calibrated REAL Jacobian',
            'height, rotation, scale and target changes are absorbed into the local scalar fit',
            'no future image sequence, actuator dynamics, IK feasibility or physical duration is simulated'],
        methods=dict(A='A initial prior fixed: 0.8 / first image mm_per_px; native A can later replace it with local feedback',
            B='mean local px/FK-mm response from other stored contexts; diagnostic transfer only',
            C='scalar DLS with numerical lambda=abs(initial prior gain)',
            D='error-dependent gain = prior * (1 + abs(current error)/abs(initial error))',
            E='A correction law with separately benchmarked global-scan/ROI feature candidate'),
        test_target_px=test_target_px,iteration_budget=iteration_budget,
        criterion_role='numerical comparison fixture only; no task tolerance or physical gate',
        executable=False,motor_write_authorized=False)
