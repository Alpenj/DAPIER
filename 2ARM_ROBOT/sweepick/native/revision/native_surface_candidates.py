"""Current depth-only surface candidates; every input pixel is retained in exactly one leaf."""
import numpy as np


def surface_candidates(seed, xyz, faces, minimum_points=3):
    original=np.asarray(seed,bool)
    pending=[original.copy()];leaves=[];splits=[]
    while pending:
        mask=pending.pop(0);v,u=np.nonzero(mask)
        points=xyz[mask]
        labels=faces.competing_depth_surface_partition(points,points[:,2],np.column_stack([u,v]),minimum_points)
        if labels is None:
            leaves.append(mask);continue
        children=[]
        for group in (0,1):
            child=np.zeros_like(mask);child[v[labels==group],u[labels==group]]=True
            assert minimum_points<=child.sum()<mask.sum()
            children.append(child)
        splits.append(dict(parent_points=int(mask.sum()),children_points=[int(c.sum()) for c in children],basis='existing spatial discontinuity + one/two-plane BIC; independent of CAD and labels'))
        pending.extend(children)
    total=np.zeros(original.shape,np.uint32)
    for mask in leaves:total+=mask
    assert np.array_equal(total,original.astype(np.uint32))
    return leaves,splits
