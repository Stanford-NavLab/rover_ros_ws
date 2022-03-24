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
    
    print(" K: ", np.round(K,2))
    print(" ----------------")
    print(" x error: ", err[0])
    print(" y error: ", err[1])
    print(" theta error: ", err[2])
    print(" v error: ", err[3])
    print(" ----------------")
    print(" u_a x contribution: ", K[1][0]*err[0])
    print(" u_a y contribution: ", K[1][1]*err[1])
    print(" u_a theta contribution: ", K[1][2]*err[2])
    print(" u_a v contribution: ", K[1][3]*err[3])
    print(" ----------------")
    print(" u_w x contribution: ", K[0][0]*err[0])
    print(" u_w y contribution: ", K[0][1]*err[1])
    print(" u_w theta contribution: ", K[0][2]*err[2])
    print(" u_w v contribution: ", K[0][3]*err[3])
    print(" ----------------")

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
    if np.abs(omega) < 0.2:
        return 0.375 * omega 

    # Positive
    elif omega > 0:

        # clip desired velocity to tested range (0 to 3.7429 rad/s)
        omega = np.clip(omega, 0.0, 3.7429)

        # use a polynomial fit to data
        p = [0.0166, -0.0466, 0.1826, 0.1150]
        pwm = 0.85 * np.polyval(p, omega)

        # clip to between 0 and 1
        return np.clip(pwm, 0.0, 1.0)

    # Negative
    else:
        omega = -omega
        omega = np.clip(omega, 0.0, 3.7429)

        p = [0.0166, -0.0466, 0.1826, 0.1150]
        pwm = 0.85 * np.polyval(p, omega)

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
    if np.abs(v) < 0.2:
        return 0.8 * v

    # clip desired velocity to tested range (0 to 1.12 m/s)
    v = np.clip(v, 0.0, 1.12)

    # use a polynomial fit to data
    p = [1.5493, -2.1160, 1.3273, -0.0069]
    pwm = 0.8 * np.polyval(p, v)

    # clip to between 0 and 1
    return np.clip(pwm, 0.0, 1.0)


def EKF_prediction_step(x_hat, u, P, A, Q, dt):
    """Perform EKF prediction step
    
    Parameters
    ----------
    x_hat: np.array (4x1)
        estimated state
    u: np.array (2x1)
        total control input
    P: np.array (4x4)
        state estimation covariance matrix
    A: np.array (4x4)
        linearized motion model matrix
    Q: np.array (4x4)
        motion model covariance
    dt: float
        discrete time-step

    Returns
    -------
    x_pred: np.array (4x1)
        predicted state
    P_pred: np.array (4x4)
        predicted state estimation covariance matrix
    """
    #compute predicted state
    x_pred = x_hat + np.array([ [x_hat[3,0]*np.cos(x_hat[2,0])], [x_hat[3,0]*np.sin(x_hat[2,0])], [u[0,0]], [u[1,0]] ])*dt;
    #compute predicted state estimation covariance matrix
    P_pred = A @ P @ A.T + Q

    return [x_pred, P_pred]


def wrap_angle(angle):
    """Wrap an angle to -pi to pi
    """
    #return (angle + np.pi) % 2*np.pi - np.pi 
    return np.arctan2(np.sin(angle), np.cos(angle))

