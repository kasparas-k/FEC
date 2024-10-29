import numpy as np

from fec._fec import _FEC

def FEC(xyz, min_component_size, tolerance, max_n, offset=True):
    """
    Fast Euclidean Clustering of a 3D point cloud.

    Args:
        xyz (np.ndarray): xyz points of the point cloud to be clustered
        min_component_size (int): ...
        tolerance (float): sphere radius defining a point's neighborhood
        max_n (int): maximum number of neighborhood points returned in nearest neighbor query
        offset (bool): apply offset so the lowest coordinate in each axis is 0 to avoid overflow

    Returns:
        list[int]: cluster index for every point 
    
    Reference:
        https://arxiv.org/abs/2208.07678
    """
    if not offset:
        flat_xyz = xyz.flatten()
    else:
        flat_xyz = (xyz - np.min(xyz, axis=0)).flatten()
    return _FEC(flat_xyz, min_component_size, tolerance, max_n)
