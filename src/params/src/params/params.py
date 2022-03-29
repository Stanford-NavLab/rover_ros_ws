import numpy as np
import math
from scipy.stats.distributions import chi2

# Planner params
DT = 0.2  # time discretization
T_SEG = 3.0  # time duration for each trajectory segment
SEG_LEN = int(T_SEG / DT)  # timesteps in a trajectory segment
T_REPLAN = 2.0  # time between planning instances
MAX_ACC_MAG = 0.25  # maximum acceleration magnitude

KW_LIMS = [-0.5, 0.5]  # limits for desired angular velocity
KV_LIMS = [-0.5, 0.5]  # limits for desired speed
SAMPLE_METHOD = 2  # 1: use covariance output by network
                   # 2: use covariance specified in params
SAMPLE_STD_DEV = 0.5  # standard deviation for sampling trajectory 
                      # [only used is sampleMethod above is selected as 2]

SIGMA_CONF_LVL = 3  # confidence level for safety
CONF_VALUE = np.sqrt(chi2.ppf(math.erf(SIGMA_CONF_LVL/np.sqrt(2)),df=2))

MAX_ORDER_INIT_REACH_SET = 10  # maximum order of reach set at the beginning of each planning segment

CALIBRATION_ITERATIONS = 10  # number of trajectory sample + safety check iterations to run during calibration
CALIBRATION_MAX_TIME_MULTIPLIER = 1.5  # multiplier for estimating max time during calibration

COLLISION_CHECK_DIST_THRESH = np.inf  # distance threshold for nearby obstacles to check
CHECK_DIST_REQ = not np.isinf(COLLISION_CHECK_DIST_THRESH)  # boolean indicating if above threshold is non-inf
COLLISION_CHECK_ZONOTOPE_ORDER = 4  # max order of confidence zonotope for collision-checking

# Controller params
Q_LQR = np.diag([5, 5, 10, 100])  # LQR state cost matrix
R_LQR = np.diag([100, 100])  # LQR control cost matrix
Q_EKF = np.diag([0.01, 0.01, 0.05, 0.01])  # EKF process noise covariance
R_EKF = np.diag([0.01, 0.01, 0.001])  # EKF measurement noise covariance (for mocap)

X_0 = np.array([-5, 0, 0, 0]).reshape((4,1))  # Initial robot state
P_0 = 0.01 * np.diag(np.array([0.01, 0.01, 0.001, 0.0]))  # Initial state estimation covariance

# Environment
OBST_ARR = np.array([0, 0, 1, 2]).reshape((4,1))  # rectangular region: cx, cy, h, w
GOAL_ARR = np.array([5, 0, 2, 1]).reshape((4,1))  # cx, cy, h, w
BIAS_ARR = np.array([0, -2, 1, 1]).reshape((4,1))  # cx, cy, h, w
BIAS_MAX_VAL = 0.1  # value of max bias in positioning (each axis) measurement

MODEL_NAME = "unicycle_reach_goal"