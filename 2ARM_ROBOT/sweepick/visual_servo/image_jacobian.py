"""Local image response on the observed motion subspace; no physical acceptance thresholds."""
import copy

import numpy as np


def matrix_diagnostics(matrix):
    a=np.asarray(matrix,float)
    if a.ndim!=2 or min(a.shape)==0 or not np.isfinite(a).all():
        raise ValueError('finite nonempty Jacobian matrix required')
    singular=np.linalg.svd(a,compute_uv=False)
    rank=int(np.linalg.matrix_rank(a))
    condition=float(singular[0]/singular[-1]) if rank==min(a.shape) and singular[-1]>0 else None
    return dict(rank=rank,singular_values=singular.tolist(),condition_number=condition,
        condition_number_status='FINITE' if condition is not None else 'UNBOUNDED',
        numerical_rank_rule='NumPy float64 SVD machine-precision rule; not a sensor/noise or safety limit',
        relative_roundoff_amplification=None if condition is None else float(condition*np.finfo(float).eps))


def jacobian_record(matrix, *, feature_axes, motion_basis_model, source, context,
                    feature_definitions, source_sha256, validated=False):
    """A MODEL matrix and an independently validated REAL matrix stay distinct."""
    a=np.asarray(matrix,float);basis=np.asarray(motion_basis_model,float)
    diagnostic=matrix_diagnostics(a)
    if source not in ('REAL_MEASURED','MODEL_ONLY_JACOBIAN','TEST_FIXTURE'):
        raise ValueError('Jacobian provenance required')
    if (a.shape[1] not in (1,2) or len(feature_axes)!=a.shape[0] or len(set(feature_axes))!=len(feature_axes)
            or not set(feature_axes)<={'u','v'} or basis.shape!=(3,a.shape[1])
            or not np.isfinite(basis).all() or np.linalg.matrix_rank(basis)<basis.shape[1]):
        raise ValueError('feature axes and independent model-frame motion basis must match the matrix')
    if not np.allclose(basis.T@basis,np.eye(basis.shape[1])):
        raise ValueError('motion basis must have unit orthogonal columns; coordinates are millimetres')
    return dict(schema='sweepick.b18.image-jacobian/1',matrix_px_per_mm=a.tolist(),
        feature_axes=list(feature_axes),motion_basis_model=basis.tolist(),
        feature_definitions=copy.deepcopy(feature_definitions),context=copy.deepcopy(context),
        source=source,source_sha256=copy.deepcopy(source_sha256),
        validated=bool(validated) and source=='REAL_MEASURED',model_only=source!='REAL_MEASURED',
        physical_match_verified=bool(validated) and source=='REAL_MEASURED',
        diagnostics=diagnostic,executable=False,motor_write_authorized=False)


def estimate_jacobian(delta_tool_mm, delta_features_px, *, feature_axes, motion_basis_model,
                      context, feature_definitions, source_sha256, source='REAL_MEASURED',
                      comparable_context=False):
    """Least squares only on excited directions; residuals do not establish sensor accuracy."""
    x=np.asarray(delta_tool_mm,float);s=np.asarray(delta_features_px,float)
    if (x.ndim!=2 or x.shape[1] not in (1,2) or s.ndim!=2 or x.shape[0]!=s.shape[0] or x.shape[0]==0
            or s.shape[1]!=len(feature_axes) or not np.isfinite(x).all() or not np.isfinite(s).all()):
        raise ValueError('paired finite NxD motion and NxF feature arrays required')
    basis=np.asarray(motion_basis_model,float)
    if basis.shape!=(3,x.shape[1]) or not np.isfinite(basis).all() or not np.allclose(basis.T@basis,np.eye(x.shape[1])):
        raise ValueError('orthonormal motion-plane basis required')
    _,singular,vt=np.linalg.svd(x,full_matrices=False);rank=int(np.linalg.matrix_rank(x))
    audit=dict(candidate_pair_count=len(x),valid_pair_count=len(x) if comparable_context else 0,
        independent_motion_directions=rank if comparable_context else 0,diagnostic_input_rank=rank,
        motion_singular_values_mm=singular.tolist(),requested_motion_dimensions=x.shape[1],
        context_comparable=bool(comparable_context),source_sha256=copy.deepcopy(source_sha256),
        measurement_noise_px=None,noise_status='not estimated without repeated independent measurements',
        physical_match_verified=False)
    if rank==0 or not comparable_context:
        return dict(status='INSUFFICIENT_PAIRED_OBSERVATIONS',jacobian=None,**audit)
    subspace=np.eye(x.shape[1]) if rank==x.shape[1] else vt[:rank].T
    active=x@subspace
    coefficients,_,_,_=np.linalg.lstsq(active,s,rcond=None)
    residual=s-active@coefficients
    record=jacobian_record(coefficients.T,feature_axes=feature_axes,
        motion_basis_model=basis@subspace,source=source,context=context,
        feature_definitions=feature_definitions,source_sha256=source_sha256,
        validated=False)
    audit.update(residuals_px=residual.tolist(),residual_rms_px=np.sqrt(np.mean(residual**2,axis=0)).tolist(),
        residual_degrees_of_freedom=int(len(x)-rank),
        observed_input_subspace_coordinates=subspace.tolist())
    return dict(status='OBSERVED_SUBSPACE_FIT_REQUIRES_VALIDATION',jacobian=record,**audit)


def solve_correction(jacobian,error_px,*,damping=0.0):
    """Return coefficients on an observable subspace, with no clipping or travel cap."""
    a=np.asarray(jacobian,float);error=np.asarray(error_px,float)
    diagnostic=matrix_diagnostics(a)
    if error.shape!=(a.shape[0],) or not np.isfinite(error).all():
        raise ValueError('finite image error matching Jacobian rows required')
    if isinstance(damping,bool) or not np.isfinite(damping) or damping<0:
        raise ValueError('finite nonnegative numerical damping required')
    _,_,vt=np.linalg.svd(a,full_matrices=False);rank=diagnostic['rank']
    if rank==0:
        return dict(status='UNOBSERVABLE',control_dimensions=0,coefficients_mm=None,
                    expected_image_change_px=None,diagnostics=diagnostic)
    subspace=np.eye(a.shape[1]) if rank==a.shape[1] else vt[:rank].T
    active=a@subspace
    lhs=np.vstack([active,damping*np.eye(rank)]) if damping else active
    rhs=np.r_[-error,np.zeros(rank)] if damping else -error
    correction,_,_,_=np.linalg.lstsq(lhs,rhs,rcond=None)
    if not np.isfinite(correction).all():
        raise ValueError('non-finite numerical correction')
    return dict(status='CALCULATED',control_dimensions=rank,coefficients_mm=correction.tolist(),
        control_subspace_coordinates=subspace.tolist(),expected_image_change_px=(active@correction).tolist(),
        diagnostics=diagnostic,damping_px_per_mm=float(damping),
        damping_role='caller numerical regularization, not a physical limit',clipping_applied=False)
