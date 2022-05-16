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
    #   such that negative error implies we should apply negative PWM
    err = x_hat - x_nom
    # Wrap theta 
    err[2] = wrap_angle(err[2])
    
    print(" - x_nom: ", np.round(x_nom[0],2), " y_nom: ", np.round(x_nom[1],2))
    print(" - x_err: ", np.round(err[0],2), " y_err: ", np.round(err[1],2))
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
    pwm = polyval2d(v, w, C)

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

