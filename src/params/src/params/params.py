import numpy as np
import math
from scipy.stats.distributions import chi2

from planner.probabilistic_zonotope import pZ

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
# Q_LQR = np.diag([50, 50, 50, 150])  # LQR state cost matrix
# R_LQR = np.diag([50, 50])  # LQR control cost matrix

# from simulation
# Q_LQR = np.diag([5, 5, 10, 100])  # LQR state cost matrix
# R_LQR = np.diag([10, 1])  # LQR control cost matrix
Q_LQR = np.diag([1, 1, 5, 50])  # LQR state cost matrix
R_LQR = np.diag([5, 1])  # LQR control cost matrix


# from initial flight room tests
#Q_EKF = np.diag([0.01, 0.01, 0.05, 0.01])  # EKF process noise covariance
#R_EKF = np.diag([0.01, 0.01, 0.001])  # EKF measurement noise covariance (for mocap)
#R_EKF = np.diag([0.001, 0.001, 0.0001])

# from original simulation
Q_EKF = np.diag([0.0001, 0.0001, 0.0005, 0.0001])  # EKF process noise covariance
R_EKF = np.diag([0.01, 0.01, 0.001])  # EKF measurement noise covariance (for mocap)

X_0 = np.array([-5, 0, 0, 0]).reshape((4,1))  # Initial robot state
P_0 = 0.01 * np.diag(np.array([0.01, 0.01, 0.001, 0.0]))  # Initial state estimation covariance

# Environment

# In-distribution (uncertainty)
OBST_ARR_1 = np.array([1.0, 0.75, 0.895, 0.79]).reshape((4,1))  # rectangular region: cx, cy, h, w
OBST_ARR_2 = np.array([-1.0, -0.75, 0.62, 0.79]).reshape((4,1))  # rectangular region: cx, cy, h, w
#OBST_ARR_1 = np.array([-1.88590713, 0.73733016, 0.80292901, 0.8556909]).reshape((4,1))  # rectangular region: cx, cy, h, w
#OBST_ARR_2 = np.array([2.01000525, -0.89226125, 0.9561391, 0.93830662]).reshape((4,1))  # rectangular region: cx, cy, h, w
GOAL_ARR = np.array([5, 0, 2, 1]).reshape((4,1))  # cx, cy, h, w
BIAS_ARR = np.array([0, 0, 20, 20]).reshape((4,1))  # cx, cy, h, w
BIAS_MAX_VAL = 0.1  # value of max bias in positioning (each axis) measurement

# Store above info in dictionary
ENV_INFO = {}
ENV_INFO['goalZ'] = pZ(GOAL_ARR[0:2,[0]], np.diag(GOAL_ARR[[3,2],0]), np.zeros((2,2)))
ENV_INFO['obstZ'] = [pZ(OBST_ARR_1[0:2,[0]], np.diag(OBST_ARR_1[[3,2],0]), np.zeros((2,2))),
                     pZ(OBST_ARR_2[0:2,[0]], np.diag(OBST_ARR_2[[3,2],0]), np.zeros((2,2)))]
# Rectangular area where bias is different: x_min, y_min, x_max, y_max
ENV_INFO['bias_area_lims'] = [BIAS_ARR[0,0]-BIAS_ARR[3,0], 
                              BIAS_ARR[1,0]-BIAS_ARR[2,0], 
                              BIAS_ARR[0,0]+BIAS_ARR[3,0], 
                              BIAS_ARR[1,0]+BIAS_ARR[2,0]]
ENV_INFO['regular_bias'] = 0.0; ENV_INFO['different_bias'] = BIAS_MAX_VAL

LANDMARK_POS = np.array([1.1563, -0.7381])  # Landmark ground-truth position [m]
LM_BOX_W = 1.0  # Width of landmark search region box [m]
LM_BOX_XMAX = LANDMARK_POS[0] + LM_BOX_W / 2  
LM_BOX_XMIN = LANDMARK_POS[0] - LM_BOX_W / 2
LM_BOX_YMAX = LANDMARK_POS[1] + LM_BOX_W / 2
LM_BOX_YMIN = LANDMARK_POS[1] - LM_BOX_W / 2
LIDAR_HEIGHT = 0.15  # Height of LiDAR off the ground [m]
LIDAR_OFFSET = np.array([0.09, 0.0])  # x-y offset of LiDAR with respect to robot center

MAX_SEGMENTS = 20

MODEL_NAME = "unicycle_2_tough_obst_kv_0.5_kw_0.5_dist_kw_reward_small_goal_best"
