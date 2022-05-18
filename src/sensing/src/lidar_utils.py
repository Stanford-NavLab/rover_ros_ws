"""LiDAR utils

"""

import numpy as np

import params.params as params


def dist_filter(P, threshold):
    """Filter points by distance
    
    Parameters
    ----------
    P : np.array (n_pts x 3)
        Point cloud to filter

    Parameters
    ----------
    np.array (n_pts_filtered x 3)
        Filtered point cloud
    
    """
    dists = np.linalg.norm(P, axis=1)
    keep_idx = dists < threshold
    return P[keep_idx,:] 


def rotate_points(P, theta):
    """Rotate 2D points by theta

    Parameters
    ----------
    P : np.array (n_pts x 2)
        2D points
    theta : float
        Angle to rotate in radians
    
    """
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta), np.cos(theta)]])
    return (R @ P.T).T


def detect_landmark(P, x_hat, d_thresh=6.0):
    """Detect landmark

    Given point cloud and state estimate, detect landmark position in robot local frame.
    
    Parameters
    ----------
    P : np.array (n_pts x 3)
        3D point cloud
    x_hat : np.array (4 x 1)
        State estimate (x, y, theta, v)
    
    Returns
    -------
    np.array (3)
        Estimated 3D landmark position in local frame
    
    """
    # Distance filter  NOTE: may not be needed anymore
    P = dist_filter(P, d_thresh)
    # Ground removal
    P = P[P[:,2] > -params.LIDAR_HEIGHT,:] 
    
    # Transform to global frame in 2D
    P_2D = P[:,:2]
    P_2D_global = rotate_points(P_2D, x_hat[2])
    P_2D_global = P_2D_global + x_hat[:2]
    
    # Use search region to extract landmark points in global frame
    mask = np.ones(len(P), dtype=bool)
    mask = mask & ((P_2D_global[:,0] <= params.LM_BOX_XMAX) & (P_2D_global[:,0] >= params.LM_BOX_XMIN))
    mask = mask & ((P_2D_global[:,1] <= params.LM_BOX_YMAX) & (P_2D_global[:,1] >= params.LM_BOX_YMIN))
    
    # Index back in local frame
    landmark_pts = P[mask]
    
    return np.mean(landmark_pts, axis=0)