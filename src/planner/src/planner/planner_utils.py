# Utilities for planner nodes

import rospy
import numpy as np

from planner.msg import State, Control, NominalTrajectory
from controller.controller_utils import wrap_angle


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


def generate_robot_matrices(x_nom, u_nom, Q_lqr, R_lqr, dt):
    """
    Generate A, B, C and K matrices for robot based on given nominal state and input vector.

    Parameters
    ----------
    x_nom : np.array (4x1)
        nominal state
    u_nom : np.array (2x1)
        nominal input
    Q_lqr : np.array (4x4)
        LQR state cost weight matrix
    R_lqr: np.array (2x2)
        LQR input cost weight matrix
    dt: float
        discrete time-step

    Returns
    -------
    A : np.array (4x4)
        linearized motion model matrix
    B : np.array (4x2)
        linearized control input matrix:
    C : np.array (3x4)
        measurement matrix
    K : np.array (2x4)
        control feedback gain matrix: 
    """
    # Get state dimension. 
    state_dim = x_nom.shape[0]
    input_dim = u_nom.shape[0]
    measurement_dim = 3  # assuming 2D position and heading measurement

    # Form linearized motion model matrix
    A = np.identity((state_dim))
    A[0,2] = -x_nom[3,0]*np.sin(x_nom[2,0])*dt
    A[0,3] = np.cos(x_nom[2,0])*dt
    A[1,2] = x_nom[3,0]*np.cos(x_nom[2,0])*dt
    A[1,3] = np.sin(x_nom[2,0])*dt

    # Form linearized control input matrix
    B = np.zeros((state_dim,input_dim))
    B[2,0] = dt; B[3,1] = dt

    # Form measurement matrix
    C = np.zeros((measurement_dim, state_dim))
    C[0,0] = 1; C[1,1] = 1; C[2,2] = 1

    # Compute control feedback gain matrix
    if np.abs(x_nom[3,0]) > 0.01:  # if there is sufficient speed for controllability
        K = dlqr_calculate(A, B, Q_lqr, R_lqr)
    else:
        K = np.array([ [0, 0, 1.0, 0], [0, 0, 0, 1.0] ])  # tuned from flight room tests

    return [A, B, C, K]


# DLQR implementation from  https://github.com/python-control/python-control/issues/359#issuecomment-759423706
def dlqr_calculate(G, H, Q, R):
    """
    Discrete-time Linear Quadratic Regulator calculation.
    State-feedback control  u[k] = -K*x[k]

    How to apply the function:    
        K = dlqr_calculate(G,H,Q,R)
        K, P, E = dlqr_calculate(G,H,Q,R, return_solution_eigs=True)

    Inputs:
      G, H, Q, R  -> all numpy arrays  (simple float number not allowed)
      returnPE: define as True to return Ricatti solution and final eigenvalues

    Returns:
      K: state feedback gain
      P: Ricatti equation solution
      E: eigenvalues of (G-HK)  (closed loop z-domain poles)
    """
    from scipy.linalg import solve_discrete_are, inv
    P = solve_discrete_are(G, H, Q, R)  #Solução Ricatti
    K = inv(H.T@P@H + R)@H.T@P@G    #K = (B^T P B + R)^-1 B^T P A 

    return K
