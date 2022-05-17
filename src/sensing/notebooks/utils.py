"""LiDAR utils

"""

import numpy as np
import open3d as o3d


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
    
    """
    # Distance filter
    P_filter = dist_filter(P, d_thresh)
    # Ground removal
    P_filter = P_filter[P_filter[:,2] > -lidar_height,:] 
    # DBSCAN clustering
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(P_filter)
    labels = np.array(pcd.cluster_dbscan(eps=dbscan_eps, min_points=dbscan_minpts))
    clusters = [[] for _ in range(labels.max()+1)]  # TODO: faster way to do this
    for i in range(len(labels)):
        if labels[i] >= 0:
            clusters[labels[i]].append(i)


def spherical_project(P):
    """Project 3D point cloud to 2D using spherical projection

    Parameters
    ----------
    P : np.array (n_pts x 3)
        3D point cloud
    
    Returns
    -------
    thetas : np.array (n_pts)
        Azimuth angles
    phis : np.array (n_pts)
        Elevation angles

    """
    thetas = np.arctan2(P[:,1], P[:,0])
    Rxy = np.sqrt(P[:,0]**2 + P[:,1]**2)
    phis = np.arctan2(P[:,2], Rxy)
    return thetas, phis