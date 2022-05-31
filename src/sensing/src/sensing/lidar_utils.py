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


### NOTE: FOR DEBUGGING ###
def process_points(P, x_hat, d_thresh=6.0):
    P = dist_filter(P, d_thresh)
    # Ground removal
    P = P[P[:,2] > -params.LIDAR_HEIGHT,:] 
    
    # Transform to global frame in 2D
    P_2D = P[:,:2]
    P_2D_global = rotate_points(P_2D, x_hat[2][0])
    P_2D_global = P_2D_global + x_hat[:2].flatten()
    return P_2D_global




# def detect_landmark(P, x_hat, d_thresh=10.0):
#     """Detect landmark

#     Given point cloud and state estimate, detect landmark position in robot local frame.
    
#     Parameters
#     ----------
#     P : np.array (n_pts x 3)
#         3D point cloud
#     x_hat : np.array (4 x 1)
#         State estimate (x, y, theta, v)
    
#     Returns
#     -------
#     np.array (2)
#         Estimated 2D landmark position in local frame
    
#     """
#     # Distance filter  NOTE: may not be needed anymore
#     P = dist_filter(P, d_thresh)
#     # Ground removal
#     P = P[P[:,2] > -params.LIDAR_HEIGHT,:] 
    
#     # Transform to global frame in 2D
#     P_2D = P[:,:2]
#     P_2D_global = rotate_points(P_2D, x_hat[2][0])
#     P_2D_global = P_2D_global + x_hat[:2].flatten()
    
#     # Use search region to extract landmark points in global frame
#     mask = np.ones(len(P), dtype=bool)
#     mask = mask & ((P_2D_global[:,0] <= params.LM_BOX_XMAX) & (P_2D_global[:,0] >= params.LM_BOX_XMIN))
#     mask = mask & ((P_2D_global[:,1] <= params.LM_BOX_YMAX) & (P_2D_global[:,1] >= params.LM_BOX_YMIN))

#     # TODO: handle case where mask is empty
#     if sum(mask) == 0:
#         print("No landmark points detected")

#     # Index back in local frame
#     landmark_pts = P[mask]

#     global_landmark_mean = np.mean(P_2D_global[mask], axis=0)
#     print("Landmark pts mean in global frame: ", global_landmark_mean)
    
#     return np.mean(landmark_pts[:,:2], axis=0)


# def get_pos_measurement(P, x_hat):
#     """Get position measurement
    
#     Given point cloud and state estimate, detect landmark and use relative position
#     to determine absolute position estimate.
    
#     """
#     robot_to_landmark_local = detect_landmark(P, x_hat)
#     # Rotate robot to landmark vector and offset vector to global frame
#     robot_to_landmark_global = rotate_points(robot_to_landmark_local, x_hat[2][0])
#     lidar_offset_global = rotate_points(params.LIDAR_OFFSET, x_hat[2][0])
#     return params.LANDMARK_POS - robot_to_landmark_global - lidar_offset_global


def detect_landmark(P, prev_vec, d_thresh=10.0):
    """Detect landmark

    Given point cloud (in robot local frame) detect landmark position in robot local frame.
    Use previous relative landmark vector to determine search area within point cloud to
    extract points from. 
    
    Parameters
    ----------
    P : np.array (n_pts x 3)
        3D point cloud
    prev_vec : np.array (2 x 1)
        Previous robot to landmark relative vector
    
    Returns
    -------
    np.array (2)
        Estimated 2D landmark position in local frame
    
    """
    # Distance filter  NOTE: may not be needed anymore
    P = dist_filter(P, d_thresh)
    # Ground removal
    P = P[P[:,2] > -params.LIDAR_HEIGHT,:] 
    
    # Extract (x,y) 2D points
    P_2D = P[:,:2]

    # Form search region from previous vector
    search_xmax = prev_vec[0] + params.LM_BOX_W / 2
    search_xmin = prev_vec[0] - params.LM_BOX_W / 2
    search_ymax = prev_vec[1] + params.LM_BOX_W / 2
    search_ymin = prev_vec[1] - params.LM_BOX_W / 2
    
    # Use search region to extract landmark points in global frame
    mask = np.ones(len(P), dtype=bool)
    mask = mask & ((P_2D[:,0] <= search_xmax) & (P_2D[:,0] >= search_xmin))
    mask = mask & ((P_2D[:,1] <= search_ymax) & (P_2D[:,1] >= search_ymin))

    # TODO: handle case where mask is empty
    if sum(mask) == 0:
        print("No landmark points detected")

    # Extract points
    landmark_pts = P_2D[mask]
    
    return np.mean(landmark_pts, axis=0)


def get_pos_measurement(robot_to_landmark_local, x_hat):
    """Get position measurement
    
    Determine absolute position estimate from relative robot to landmark 
    vector and state estimate.
    
    """
    # Rotate robot to landmark vector and offset vector to global frame
    robot_to_landmark_global = rotate_points(robot_to_landmark_local, x_hat[2][0])
    lidar_offset_global = rotate_points(params.LIDAR_OFFSET, x_hat[2][0])
    return params.LANDMARK_POS - robot_to_landmark_global - lidar_offset_global