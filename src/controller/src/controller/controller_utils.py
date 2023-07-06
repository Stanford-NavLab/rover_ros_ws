# Utilities for controller nodes

import rospy
import numpy as np
from numpy.polynomial.polynomial import polyval2d

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
    err = x_hat - x_nom
    err[2] = wrap_angle(err[2])  # Wrap theta 

    # Compute total control input
    u = u_nom - K @ err
    print("err: ", err)
    print("K @ err: ", K @ err)

    return u


def ang_PWM(v, w):
    """Mapping from desired linear velocity (m/s) and angular
    velocity (rad/s) to angular PWM

    Parameters
    ----------
    v : float
        desired linear velocity
    w : float
        desired angular velocity
    
    Returns
    -------
    pwm : float
        pwm value between -1 and 1

    """
    # Polynomial fit from data
    # Coefficients of p(x,y) = c00 + c10*x + c01*y + c20*x^2 + c11*x*y + c02*y^2 + c30*x^3 
    #                          + c21*x^2*y + c12*x*y^2 + c03*y^3
    C = np.array([[-0.00581, 0.2623, -0.004842, -0.06905],
                  [0.02074, -0.2124, 0.02372, 0.0],
                  [-0.06311, 0.3288, 0.0, 0.0],
                  [0.06762, 0.0, 0.0, 0.0]])
    pwm = polyval2d(v, w, C)

    # clip to between -1 and 1
    return np.clip(pwm, -1.0, 1.0)


def lin_PWM(v, w):
    """Mapping from desired linear velocity (m/s) and angular
    velocity (rad/s) to linear PWM

    Parameters
    ----------
    v : float
        desired linear velocity
    w : float
        desired angular velocity
    
    Returns
    -------
    pwm : float
        pwm value between 0 and 1

    """
    # Polynomial fit from data
    # Coefficients of p(x,y) = c00 + c10*x + c01*y + c20*x^2 + c11*x*y + c02*y^2 + c30*x^3 
    #                          + c21*x^2*y + c12*x*y^2 + c03*y^3
    C = np.array([[0.1181, 0.007321, -0.05408, -0.004855],
                  [0.1276, -0.03117, 0.1554, 0.0],
                  [0.7645, 0.08861, 0.0, 0.0],
                  [-0.691, 0.0, 0.0, 0.0]])

    if v < 0:
        pwm = -polyval2d(-v, w, C)
    else: 
        pwm = polyval2d(v, w, C)

    # clip to between -1 and 1
    return np.clip(pwm, -1.0, 1.0)


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
    # Compute predicted state
    x_pred = x_hat + np.array([ [x_hat[3,0]*np.cos(x_hat[2,0])], [x_hat[3,0]*np.sin(x_hat[2,0])], [u[0,0]], [u[1,0]] ])*dt;
    # Compute predicted state estimation covariance matrix
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
    # Compute Kalman gain
    L = P_pred @ C.T @ np.linalg.inv(C @ P_pred @ C.T + R)
    # Compute corrected state estimate
    x_hat = x_pred + L @ (z - C @ x_pred)
    # Compute corrected state estimation covariance matrix
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

