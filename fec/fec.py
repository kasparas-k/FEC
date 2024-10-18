import numpy as np

from fec._fec import _FEC

def FEC(xyz, min_component_size, tolerance, max_n):
    """
    Fast Euclidean Clustering of a 3D point cloud.

    Args:
        xyz (np.ndarray): xyz points of the point cloud to be clustered
        min_component_size (int): ...
        tolerance (float): sphere radius defining a point's neighborhood
        max_n (int): maximum number of neighborhood points returned in nearest neighbor query

    Returns:
        list[int]: cluster index for every point 
    
    Reference:
        https://arxiv.org/abs/2208.07678
    """
    return _FEC(list(xyz.flatten()), min_component_size, tolerance, max_n)
