import rospy
import rospkg
import numpy as np
import time 

from trajectory_msgs.msg import JointTrajectory

import params.rtd_params as params
from rtd.LPM import LPM
import rtd.utils as utils


class LinearPlanner:
    """Linear Planner class

    Linear Trajectory planner which recomputes collision-free trajectories to follow 
    in a receding-horizon fashion.

    Trajectory: time-indexed positions, velocities, accelerations
    Plan: trajectory and associated reachable set(s)
    
    Attributes
    ----------

    Methods
    -------

    """
    def __init__(self):
        # Initialize node
        rospy.init_node('linear_planner', anonymous=True, disable_signals=True)
        self.rate = rospy.Rate(10)

        # Initialize LPM object
        lpm_file = rospkg.RosPack().get_path('rtd') + '/models/' + params.MODEL_NAME
        self.LPM = LPM(lpm_file)
        self.N_T_PLAN = len(self.LPM.time)  # planned trajectory length
        self.DT = self.LPM.t_sample  # trajectory discretization time interval

        # Replan timer
        rospy.Timer(rospy.Duration(params.T_REPLAN), self.replan)

        # Publishers
        self.traj_pub = rospy.Publisher('planner/traj', JointTrajectory, queue_size=10)

        # Initial conditions [m],[m/s],[m/s^2]
        self.p_0 = params.P_0
        self.v_0 = np.zeros((params.N_DIM,1))
        self.a_0 = np.zeros((params.N_DIM,1))

        # Goal position [m]
        self.p_goal = params.P_GOAL

        # Obstacles
        self.obstacles = params.OBSTACLES

        self.done = False


    def check_obstacle_collisions(self, positions):
        """ Check a sequence of positions against the current list of nearby obstacles for collision.

        Parameters
        ----------
        positions : np.array
            Trajectory positions to check against obstacles.

        Returns
        -------
        bool
            True if plan is safe, False if there is a collision.

        """
        for obs in self.obstacles:
            if not utils.check_obs_collision(positions, obs, 2*params.R_BOT):
                return False
        return True


    def traj_opt(self, t_start_plan):
        """Trajectory Optimization

        Attempt to find a collision-free plan (v_peak) which brings the agent 
        closest to its goal.

        Parameters
        ----------
        t_start_plan : float
            Time at which planning started for this iteration

        Returns
        -------
        np.array or None
            Optimal v_peak or None if failed to find one
        
        """
        # Generate potential v_peak samples
        V_peak = utils.rand_in_bounds(params.V_BOUNDS, params.N_PLAN_MAX)
        # Eliminate samples that exceed the max velocity and max delta from initial velocity
        V_peak = utils.prune_vel_samples(V_peak, self.v_0, params.V_MAX_NORM, params.DELTA_V_PEAK_MAX)

        # Calculate the endpoints for the sample v_peaks
        P_endpoints = self.LPM.compute_endpoints(self.v_0, self.a_0, V_peak) + self.p_0
        
        # Sort V_peaks by distance to goal
        dist_to_goal = np.linalg.norm(P_endpoints - self.p_goal, axis=0)
        V_sort_idxs = np.argsort(dist_to_goal)
        V_peak = V_peak[:,V_sort_idxs]

        # Iterate through V_peaks until we find a feasible one
        for i in range(V_peak.shape[1]):
        
            # Get candidate trajectory positions for current v_peak
            v_peak = V_peak[:,i][:,None]
            k = np.hstack((self.v_0, self.a_0, v_peak))
            cand_traj = self.LPM.compute_positions(k) + self.p_0

            # check against obstacles
            check_obs = self.check_obstacle_collisions(cand_traj)

            if check_obs:
                return v_peak

            if (time.time() - t_start_plan > params.T_PLAN):
                print("Ran out of time for planning, idx = ", i)
                break

        # No v_peaks are feasible (or we ran out of time)
        return None


    def replan(self, event):
        """Replan

        Periodically called to perform trajectory optimization.
        
        """
        t_start_plan = time.time()

        # Find a new v_peak
        v_peak = self.traj_opt(t_start_plan)

        if v_peak is None:
            # Failed to find new plan
            print("Failed to find new plan")
        else:
            # Generate new trajectory
            k = np.hstack((self.v_0, self.a_0, v_peak))
            P,V,A = self.LPM.compute_trajectory(k)
            P = P + self.p_0  # translate to p_0

            # Update initial conditions
            self.v_0 = V[:,params.NEXT_IC_IDX][:,None]
            self.a_0 = A[:,params.NEXT_IC_IDX][:,None]
            self.p_0 = P[:,params.NEXT_IC_IDX][:,None]

            print("Found new trajectory, v_pk = ", np.round(k[:,2], 2))
            print(" Start point: ", np.round(P[:,0], 2))
            print(" End point: ", np.round(P[:,-1], 2))

            # Create and send trajectory msg
            t2start = 0  # TODO: this is just a filler value for now
            traj_msg = utils.wrap_2D_traj_msg((P,V,A), t2start)
            self.traj_pub.publish(traj_msg)
        
        # Check for goal-reached
        if np.linalg.norm(P[:,-1][:,None] - self.p_goal) < params.R_GOAL_REACHED:
            print("Goal reached")
            self.done = True

    
    def run(self):
        """Run node

        """
        rospy.loginfo("Running Linear Planner")

        while not rospy.is_shutdown() and not self.done:
            # connections = self.traj_pub.get_num_connections()
            # rospy.loginfo("Waiting for tracker, connections: %d", connections)

            # if connections > 0:
            
            # loop while replan is periodically called by Timer

            self.rate.sleep()

        rospy.loginfo("Exiting node")


if __name__ == '__main__':
    lp = LinearPlanner()
    try:
        lp.run()
    except rospy.ROSInterruptException:
        pass
