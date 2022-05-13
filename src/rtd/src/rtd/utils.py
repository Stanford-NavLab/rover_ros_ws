"""Utils

Utilities for RTD.

"""

import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class Trajectory:
    """Trajectory class

    Planned trajectory
    
    Attributes
    ----------
    length : int
        Length of trajectory (i.e. number of timesteps), abbreviated as N below
    N_dim : int
        State dimension of the trajectory (i.e. 2D or 3D)
    time : np.array (1 x N)
        Time array
    positions : np.array (N_dim x N)
        Positions
    velocities : np.array (N_dim x N)
        Velocities
    accelerations : np.array (N_dim x N)
        Accelerations

    """
    # def __init__(self, time, N_dim):
    #     """Initialize a trajectory with 0 position, velocity, and acceleration"""
    #     self.length = len(time)
    #     self.N_dim = N_dim
    #     self.time = time
    #     self.positions = np.zeros((N_dim, self.length))
    #     self.velocities = np.zeros((N_dim, self.length))
    #     self.accelerations = np.zeros((N_dim, self.length))
    def __init__(self, time, positions, velocities, accelerations):
        """Initialize a trajectory with 0 position, velocity, and acceleration"""
        self.length = len(time)
        self.N_dim = positions.shape[0]
        self.time = time
        self.positions = positions
        self.velocities = velocities
        self.accelerations = accelerations


def wrap_2D_traj_msg(traj, t2start):
    """Wraps a 2-D trajectory in a JointTrajectory message.

    Parameters
    ----------
    traj : tuple (p,v,a) of np.array (2 x N)
        Trajectory containing position, velocity, and acceleration
    t2start : float
        Time to start in seconds

    Returns
    -------
    JointTrajectory 
        Wrapped message.

    """
    p,v,a = traj
    traj_msg = JointTrajectory()

    jtp_x = JointTrajectoryPoint()
    jtp_x.positions = p[0]
    jtp_x.velocities = v[0]
    jtp_x.accelerations = a[0]
    jtp_x.time_from_start = rospy.Duration(t2start)

    jtp_y = JointTrajectoryPoint()
    jtp_y.positions = p[1]
    jtp_y.velocities = v[1]
    jtp_y.accelerations = a[1]
    jtp_y.time_from_start = rospy.Duration(t2start)

    traj_msg.points = [jtp_x, jtp_y]
    traj_msg.joint_names = ['x','y']

    return traj_msg


def unwrap_2D_traj_msg(msg, time):
    """Convert JointTrajectory message to Trajectory class

    Parameters
    ----------
    msg : JointTrajectory 
        JointTrajectory message
    time : np.array (1 x N)
        Time vector

    Returns
    -------
    Trajectory
        Trajectory wrapped in class

    """
    px = msg.points[0].positions
    py = msg.points[1].positions
    vx = msg.points[0].velocities
    vy = msg.points[1].velocities
    ax = msg.points[0].accelerations
    ay = msg.points[1].accelerations

    pos = np.vstack((px, py))
    vel = np.vstack((vx, vy))
    acc = np.vstack((ax, ay))

    return Trajectory(time, pos, vel, acc)



def check_obs_collision(positions, obs, r_collision):
    """Check a sequence of positions against a single obstacle for collision.
    Obstacles are cylinders represented as (center, radius)
    Parameters
    ----------
    positions : np.array
    obs : tuple
    Returns
    -------
    bool
        True if the plan is safe, False is there is a collision
    """
    c_obs, r_obs = obs
    d_vec = np.linalg.norm(positions - c_obs[:,None], axis=0)
    if any(d_vec <= r_collision + r_obs):
        return False
    else:
        return True


def rand_in_bounds(bounds, n):
    """Generate random samples within specified bounds
    Parameters
    ----------
    bounds : list
        List of min and max values for each dimension.
    n : int
        Number of points to generate.
    Returns
    -------
    np.array 
        Random samples
    """
    x_pts = np.random.uniform(bounds[0], bounds[1], n)
    y_pts = np.random.uniform(bounds[2], bounds[3], n)
    # 2D 
    if len(bounds) == 4:
        return np.vstack((x_pts, y_pts))
    # 3D
    elif len(bounds) == 6:
        z_pts = np.random.uniform(bounds[4], bounds[5], n)
        return np.vstack((x_pts, y_pts, z_pts))
    else:
        raise ValueError('Please pass in bounds as either [xmin xmax ymin ymax] '
                            'or [xmin xmax ymin ymax zmin zmax] ')


def prune_vel_samples(V, v_0, max_norm, max_delta):
    """Prune Velocity Samples
    
    """
    V_mag = np.linalg.norm(V, axis=0)
    delta_V = np.linalg.norm(V - v_0, axis=0)
    keep_idx = np.logical_and(V_mag < max_norm, delta_V < max_delta)
    return V[:,keep_idx]


def dlqr_calculate(G, H, Q, R):
    """
    Discrete-time Linear Quadratic Regulator calculation.
    State-feedback control  u[k] = -K*x[k]
    Implementation from  https://github.com/python-control/python-control/issues/359#issuecomment-759423706
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