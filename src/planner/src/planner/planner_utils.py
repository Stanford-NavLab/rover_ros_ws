# General utility for planner nodes

import numpy as np
import time

import planner.reachability_utils as reach_util
from controller.controller_utils import wrap_angle
import params.params as params
from planner.msg import State, Control

def wrap_states(x_nom):
    """Wraps a np array of nominal states into a vector of state msgs

    Parameters
    ----------
    x_nom : np.array (4xN where N is trajectory length)
        nominal states

    Returns
    -------
    states : State[]
        vector of state msgs
    """
    N = x_nom.shape[1]
    states = [None] * N

    for i in range(N):
        s = State()
        s.x = x_nom[0][i]
        s.y = x_nom[1][i]
        s.theta = x_nom[2][i]
        s.v = x_nom[3][i]
        states[i] = s

    return states


def wrap_controls(u_nom):
    """Wraps a np array of nominal controls into a vector of control msgs

    Parameters
    ----------
    u_nom : np.array (2xN where N is trajectory length)
        nominal controls

    Returns
    -------
    controls : Control[]
        vector of control msgs
    """
    N = u_nom.shape[1]
    controls = [None] * N
    
    for i in range(N):
        c = Control()
        c.omega = u_nom[0][i]
        c.a = u_nom[1][i]
        controls[i] = c

    return controls


def trajectory_parameter_to_nominal_trajectory(kw, kv, xnom0, t_plan, dt, max_acc_mag):
    """Map trajectory parameter to nominal trajectory

    Create nominal trajectory starting from xnom0 with parameters kw (desired angular rate) 
    and kv (desired speed) for duration t_plan seconds. A fail-safe maneuver is appended 
    after t_plan seconds.
    
    Parameters
    ----------
    kw : float
        desired angular rate: 
    kv : float
        desired speed: 
    xnom0 : np.array (4x1)
        initial nominal state (x position [m], y position [m], heading angle [rad], speed [m/s])
    t_plan : float
        total trajectory time duration
    dt : float
        discrete time-step
    max_acc_mag : float
        maximum magnitude of acceleration/deceleration [m/s^2]

    Returns
    -------
    xnom : np.array (4xN where N is trajectory length)
        nominal states
    unom : np.array (2xN where N is trajectory length)
        nominal control inputs (angular velocity [rad/s], linear acceleration [m/s^2])
    """

    # Initialize nominal trajectory arrays
    state_dim = xnom0.shape[0]
    input_dim = 2
    N_initial_timesteps = int(t_plan/dt) 
    xnom = np.zeros((state_dim, N_initial_timesteps+1))  # including xnom0
    unom = np.zeros((input_dim, N_initial_timesteps))  # yaw rate and acceleration
    xnom[:,[0]] = xnom0

    # Create the nominal trajectory
    for k in range(N_initial_timesteps):
        des_acc = (kv - xnom[3,k])/dt
        unom[1,k] = np.clip(des_acc, -max_acc_mag, max_acc_mag)
        unom[0,k] = kw
        xnom[:,[k+1]] = xnom[:,[k]] + dt*np.array([[xnom[3,k]*np.cos(xnom[2,k])], 
                                                   [xnom[3,k]*np.sin(xnom[2,k])], 
                                                   [unom[0,k]], 
                                                   [unom[1,k]]])
        xnom[2,[k+1]] = wrap_angle(xnom[2,k+1])

    # Append braking maneuver
    N_brake_timesteps = max(int(np.ceil(np.abs(xnom[3,-1])/(max_acc_mag*dt))), 1)
    xnom = np.concatenate((xnom, np.zeros((state_dim, N_brake_timesteps))), axis=1)
    unom = np.concatenate((unom, np.zeros((input_dim, N_brake_timesteps))), axis=1)

    for k in range(N_initial_timesteps, N_initial_timesteps + N_brake_timesteps):
        des_acc = (0 - xnom[3,k])/dt
        unom[1,k] = np.clip(des_acc, -max_acc_mag, max_acc_mag)
        xnom[:,[k+1]] = xnom[:,[k]] + dt*np.array([[xnom[3,k]*np.cos(xnom[2,k])], 
                                                   [xnom[3,k]*np.sin(xnom[2,k])], 
                                                   [unom[0,k]], 
                                                   [unom[1,k]]])
        xnom[2,[k+1]] = wrap_angle(xnom[2,k+1])

    # True brake
    # xnom_brake = np.zeros((state_dim, 1))  # just say all zeros for now bc lazy
    # unom_brake = np.array([[0.0], [-np.inf]]) # -np.inf is signal to controller to force stop motors
    
    # xnom = np.concatenate((xnom, xnom_brake), axis=1)
    # unom = np.concatenate((unom, unom_brake), axis=1)

    return [xnom, unom]


def check_trajectory_parameter_safety(kw, kv, x_nom0, Xaug0, P0, env):
    """Check if trajectory parameter is safe

    TODO
    
    """
    # Create nominal trajectory for given trajectory parameter
    start_time = time.time()
    [xnom_seg, unom_seg] = trajectory_parameter_to_nominal_trajectory(
        kw, kv, x_nom0, params.T_SEG, params.DT, params.MAX_ACC_MAG)
    print("Trajectory generated from parameter time: ", time.time() - start_time, " seconds")

    # Create motion and sensing pZs along nominal trajectory
    [WpZ, VpZs, Rhats] = reach_util.create_motion_sensing_pZ(
        xnom_seg, params.Q_EKF, params.R_EKF, params.SIGMA_CONF_LVL, 
        env['bias_area_lims'], env['regular_bias'], 
        env['different_bias'])

    # Compute reachable sets for the trajectory with either range sensing or position sensing
    [Xaug, Zaug, P_all] = reach_util.compute_reachable_sets_position_sensing(
        xnom_seg, unom_seg, Xaug0, P0, params.Q_EKF, WpZ, VpZs, Rhats, 
        params.Q_LQR, params.R_LQR, params.SIGMA_CONF_LVL, params.DT)

    # Check if trajectory is safe
    isSafe = reach_util.is_collision_free(Zaug, env['obstZ'], 
        collisionCheckOrder=params.COLLISION_CHECK_ZONOTOPE_ORDER, 
        distCheck=params.CHECK_DIST_REQ, 
        dist_threshold=params.COLLISION_CHECK_DIST_THRESH)

    return isSafe, Xaug, Zaug, P_all, xnom_seg, unom_seg


def is_trajectory_inside_region(x_nom, region_array):
    """Check if nominal trajectory is inside specified region

    TODO
    
    """
    N = x_nom.shape[1]  # Length of trajectory

    # Get 1D distances of points from region center and check if any are less than both the height and the width
    points_xy_dists = np.abs(x_nom[0:2,:] - np.tile(region_array[0:2,[0]], (1,N)))
    isWithinHeightWidth = points_xy_dists < np.tile(region_array[[3,2],[0]].reshape(2,1), (1,N))
    isInside = np.any(np.logical_and(isWithinHeightWidth[0,:], isWithinHeightWidth[1,:]))

    return isInside


def calibrate_sample_safety_check_time(x_nom0, Xaug0, P0, env):
    """
    TODO
    calibration process to estimate the maximum time needed for sampling a trajectory parameter and checking its safety
    """

    N = params.CALIBRATION_ITERATIONS
    iter_times = np.zeros(N)
    # calibration_params = params.copy()
    # calibration_params['collision_check_dist_threshold'] = 999
    # calibration_params['shouldCheckDist'] = True

    for i in range(N):    

        # Note start time for iteration
        t_start = time.time()

        # Sample new trajectory parameters within specified limits near network output. TODO: sample random parameters instead of the center ones
        kw, kv = sample_near_trajectory_parameters(
            (params.KW_LIMS[0]+params.KW_LIMS[1])/2, 
            (params.KV_LIMS[0]+params.KV_LIMS[1])/2)

        # Create nominal trajectory and check safety
        check_trajectory_parameter_safety(kw, kv, x_nom0, Xaug0, P0, env)

        # Note time taken for iteration
        iter_times[i] = time.time() - t_start

    # Save a conservative estimate for max time
    mean_iter_time = np.mean(iter_times)
    Delta_t = np.round(params.CALIBRATION_MAX_TIME_MULTIPLIER * mean_iter_time, 2)

    return Delta_t


def sample_near_trajectory_parameters(kw0, kv0):
    """
    TODO
    Sample trajectory parameters near original parameters within specified limits
    """
    kw = np.inf
    while kw < params.KW_LIMS[0] or kw > params.KW_LIMS[1]:
        kw = np.random.normal(kw0, params.SAMPLE_STD_DEV)
    kv = np.inf
    while kv < params.KV_LIMS[0] or kv > params.KV_LIMS[1]:
        kv = np.random.normal(kv0, params.SAMPLE_STD_DEV)

    return kw, kv


def update_nominal_trajectory(x_nom, u_nom, xnom_seg, unom_seg, segment_number):
    """
    TODO
    """
    # Append previous nominal trajectory (without fail-safe) with new segment trajectory (excluding first nominal position)
    x_nom = np.append(x_nom[:,:params.SEG_LEN*segment_number+1], xnom_seg[:,1:], axis=1)
    u_nom = np.append(u_nom[:,:params.SEG_LEN*segment_number], unom_seg, axis=1)

    return x_nom, u_nom


def sample_near_network_output(action_mean, action_cov):
    """
    TODO
    """
    kw0 = action_mean[0,0]; kv0 = action_mean[0,1]
    kw_std = action_cov[0,0]; kv_std = action_cov[1,1]
    kw = np.inf
    while kw < params.KW_LIMS[0] or kw > params.KW_LIMS[1]:
        kw = np.random.normal(kw0, kw_std)
    kv = np.inf
    while kv < params.KV_LIMS[0] or kv > params.KV_LIMS[1]:
        kv = np.random.normal(kv0, kv_std)

    return kw, kv