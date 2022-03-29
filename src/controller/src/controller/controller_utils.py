# Utilities for controller nodes

import rospy
import numpy as np

def compute_control(x_nom, u_nom, x_hat, K):
    """Compute total control input vector

    Parameters
    ----------
    x_nom : np.array (4x1)
        nominal state
    u_nom : np.array (4x1)
        nominal control input
    x_hat : np.array (4x1)
        estimated state
    K : np.array (2x4)
        control feedback gain matrix

    Returns
    -------
    u : np.array (2x1)
        total control input
    """
    # Get error between estimated and nominal states
    #   such that negative error implies we should apply negative PWM
    err = x_hat - x_nom
    # Wrap theta 
    err[2] = wrap_angle(err[2])
    
    # print(" K: ", np.round(K,2))
    # print(" x_hat: ", x_hat)
    # print(" x_nom: ", x_nom)
    # print(" ----------------")
    # print(" x error: ", err[0])
    # print(" y error: ", err[1])
    # print(" theta error: ", err[2])
    # print(" v hat: ", x_hat[3])
    # print(" v nom: ", x_nom[3])
    # print(" v error: ", err[3])
    # print(" ----------------")
    # print(" u_a x contribution: ", K[1][0]*err[0])
    # print(" u_a y contribution: ", K[1][1]*err[1])
    # print(" u_a theta contribution: ", K[1][2]*err[2])
    # print(" u_a v contribution: ", K[1][3]*err[3])
    # print(" ----------------")
    # print(" u_w x contribution: ", K[0][0]*err[0])
    # print(" u_w y contribution: ", K[0][1]*err[1])
    # print(" u_w theta contribution: ", K[0][2]*err[2])
    # print(" u_w v contribution: ", K[0][3]*err[3])
    # print(" ----------------")
    # print(" K @ err: ", K @ err)

    # Compute total control input
    u = u_nom - K @ err

    return u


def omega_to_PWM(omega):
    """Mapping from desired angular velocity (rad/s) to PWM

    Parameters
    ----------
    omega : float
        desired angular velocity
    
    Returns
    -------
    pwm : float
        pwm value between -1 and 1
    """
    # Linear zone
    if np.abs(omega) < 0.7301:
        return (0.2/0.7301) * omega 

    # Positive
    elif omega > 0:

        # clip desired velocity to tested range (0 to 4.1997 rad/s)
        omega = np.clip(omega, 0.0, 4.1997)

        # use a polynomial fit to data
        p = [0.0180, -0.0751, 0.2235, 0.0708]
        pwm = np.polyval(p, omega)

        # clip to between 0 and 1
        return np.clip(pwm, 0.0, 1.0)

    # Negative
    else:
        omega = -omega
        omega = np.clip(omega, 0.0, 4.1997)

        p = [0.0180, -0.0751, 0.2235, 0.0708]
        pwm = np.polyval(p, omega)

        return np.clip(-pwm, -1.0, 0.0)


def v_to_PWM(v):
    """Mapping from desired linear velocity (m/s) to PWM

    Parameters
    ----------
    v : float
        desired linear velocity
    
    Returns
    -------
    pwm : float
        pwm value between 0 and 1
    """
    if np.abs(v) < 0.26:
        return (0.2/0.26) * v

    # clip desired velocity to tested range (0 to 1.25 m/s)
    v = np.clip(v, 0.0, 1.25)

    # use a polynomial fit to data
    p = [1.2377, -1.9013, 1.2622, -0.0227]
    pwm = np.polyval(p, v)

    # clip to between 0 and 1
    return np.clip(pwm, 0.0, 1.0)


def EKF_prediction_step(x_hat, u, P, A, Q, dt):
    """Perform EKF prediction step
    
    Parameters
    ----------
    x_hat : np.array (4x1)
        estimated state
    u : np.array (2x1)
        total control input
    P : np.array (4x4)
        state estimation covariance matrix
    A : np.array (4x4)
        linearized motion model matrix
    Q : np.array (4x4)
        motion model covariance
    dt : float
        discrete time-step

    Returns
    -------
    x_pred : np.array (4x1)
        predicted state
    P_pred : np.array (4x4)
        predicted state estimation covariance matrix
    """
    #compute predicted state
    x_pred = x_hat + np.array([ [x_hat[3,0]*np.cos(x_hat[2,0])], [x_hat[3,0]*np.sin(x_hat[2,0])], [u[0,0]], [u[1,0]] ])*dt;
    #compute predicted state estimation covariance matrix
    P_pred = A @ P @ A.T + Q

    return x_pred, P_pred


def EKF_correction_step(x_pred, P_pred, z, C, R):
    """
    Perform EKF correction step
    
    Parameters
    ----------
    x_pred : np.array (4x1)
        predicted state
    P_pred : np.array (4x4)
        predicted state estimation covariance matrix
    z : np.array (4x1)
        received measurement 
    C : np.array (3x4)
        measurement matrix
    R : np.array (3x3)
        sensing model covariance

    Returns
    -------
    x_hat : np.array (4x1)
        corrected state estimate
    P : np.array (4x4)
        corrected state estimation covariance matrix
    L : np.array (4x3)
        Kalman gain matrix
    """
    #compute Kalman gain
    L = P_pred @ C.T @ np.linalg.inv(C @ P_pred @ C.T + R)
    # print(" L: ", L)
    # print(" z: ", z)
    # print(" C @ x_pred: ", C @ x_pred)
    # print(" correction term: ", L @ (z - C @ x_pred))
    #compute corrected state estimate
    x_hat = x_pred + L @ (z - C @ x_pred)
    #compute corrected state estimation covariance matrix
    P = P_pred - L @ C @ P_pred

    return x_hat, P


def wrap_angle(angle):
    """Wrap an angle to -pi to pi

    Parameters
    ----------
    angle : float
        angle to be wrapped

    Returns
    -------
    float
        angle wrapped to between -pi and -pi
    """
    #return (angle + np.pi) % 2*np.pi - np.pi 
    return np.arctan2(np.sin(angle), np.cos(angle))

