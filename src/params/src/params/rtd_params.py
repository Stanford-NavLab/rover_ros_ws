import numpy as np

# Timing parameters
T_REPLAN = 0.5  # [s] amount of time between replans
T_PLAN = 0.4  # [s] amount of time allotted for planning itself 
              #     (allow buffer for actual tranmission of plan)

N_DIM = 2  # workspace dimension (i.e. 2D or 3D)

R_BOT = 0.2  # [m]

# Max velocity constraints [m/s]
V_MAX = 2.0  # L1 velocity constraints
V_BOUNDS = np.tile(np.array([-V_MAX, V_MAX]), (1,N_DIM))[0]
V_MAX_NORM = 2.0  # L2 velocity constraints
DELTA_V_PEAK_MAX = 3.0  # Delta from initial velocity constraint

R_GOAL_REACHED = 0.3  # [m] stop planning when within this dist of goal

N_PLAN_MAX = 10000  # Max number of plans to evaluate

MODEL_NAME = 'quadrotor_linear_planning_model.mat'

P_0 = np.array([[-5], [0]])  # Initial position

P_GOAL = np.array([[5], [0]])  # Goal position

OBSTACLES = [(np.array([3, 1]), 1.5),
             (np.array([0, -1.5]), 1.5),
             (np.array([-3, 1]), 1.5)]