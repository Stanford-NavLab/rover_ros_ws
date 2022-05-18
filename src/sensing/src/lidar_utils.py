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


def extract_landmarks(P, d_thresh=6.0, lidar_height=0.15, dbscan_eps=0.2, dbscan_minpts=10):
    """Extract landmark positions from point cloud

    Parameters
    ----------
    P : np.array (n_pts x 3)
        3D point cloud
    d_thresh : float (meters)
        Distance threshold for filtering
    lidar_height : float (meters)
        LiDAR height (for ground plane removal)

    Returns
    -------
    
    """
    # Distance filter
    P_filter = dist_filter(P, d_thresh)
    # Ground removal
    P_filter = P_filter[P_filter[:,2] > -lidar_height,:] 
    # DBSCAN clustering
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(P_filter)
    labels = np.array(pcd.cluster_dbscan(eps=dbscan_eps, min_points=dbscan_minpts))
    clusters = [[] for _ in range(labels.max()+1)]  # TODO: faster way to do this?
    for i in range(len(labels)):
        if labels[i] >= 0:
            clusters[labels[i]].append(i)