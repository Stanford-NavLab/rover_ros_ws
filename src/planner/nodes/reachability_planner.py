#!/usr/bin/env python

import rospy
import numpy as np
import rospkg
import matplotlib.pyplot as plt

from planner.msg import State, Control, NominalTrajectory
import planner.planner_utils as plan_util
import planner.reachability_utils as reach_util
import planner.NN_utils as nn_util
from planner.probabilistic_zonotope import pZ
import params.params as params


class nn_planner():
    """Neural Network Planner

    Planner which uses a trained RL (reinforcement learning) model to generate
    nominal trajectories.

    """
    def __init__(self):
        # Initialize node 
        rospy.init_node('nn_planner', anonymous=True, disable_signals=True)
        self.rate = rospy.Rate(10)

        # Publishers
        self.traj_pub = rospy.Publisher('planner/traj', NominalTrajectory, queue_size=10)

        # Replan timer
        rospy.Timer(rospy.Duration(params.T_SEG), self.replan)
        self.init_time = rospy.get_time()

        # Class variables
        self.x_nom0 = params.X_0
        self.P0 = params.P_0
        self.traj_msg = None

        # Load learned model
        print("Loading model")
        model_file = rospkg.RosPack().get_path('planner') + '/models/' + params.MODEL_NAME
        self.model = nn_util.load_model(model_file)

        # Get initial reach set
        self.Xaug0 = reach_util.initialize_reachability_analysis(params.X_0, params.P_0)

        # Calibration to estimate max time it would take to sample a trajectory parameter and check its safety
        self.max_check_time = plan_util.calibrate_sample_safety_check_time(
            params.X_0, self.Xaug0, params.P_0, params.ENV_INFO)
        print("Calibrating max safety check time: ", self.max_check_time, " s")

        self.done = False  # flag to check when to stop planning
        self.seg_num = 0  # current segment number

        #complete x_nom and u_nom
        # x_nom_RR = x_nom0; u_nom_RR = np.zeros((2,0))
        # # Complete confidence zonotopes
        # Z_aug_RR = [pZ.conf_zonotope(pZ(self.Xaug0.c[0:2,:], self.Xaug0.G[0:2,:], self.Xaug0.Sigma[0:2,0:2]), params.CONF_VALUE)]
        

    def replan(self, event):
        """Plan next trajectory segment

        """
        print("Replanning: t = ", rospy.get_time() - self.init_time)

        #reduce initial reach set to specified order
        self.Xaug0 = pZ.reduce(self.Xaug0, params.MAX_ORDER_INIT_REACH_SET)

        # Get network output
        [action_mean, action_cov] = nn_util.evaluate_model(self.model, self.x_nom0)
        kw0 = action_mean[0,0]; kv0 = action_mean[0,1]

        # Check if trajectory specified by above nominal trajectory is safe (fail-safe trajectory is appended)
        [safeTrajectoryFound, Xaug, Zaug, P_all, xnom_seg, unom_seg] = plan_util.check_trajectory_parameter_safety(kw0, kv0, self.x_nom0, self.Xaug0, self.P0, params.ENV_INFO)

        # If network output trajectory was safe
        if safeTrajectoryFound:
            # Select network output as trajectory parameter
            kw_safe = kw0; kv_safe = kv0

            # Store final nominal state, reachable set, state estimation covariance
            self.x_nom0 = xnom_seg[:,[params.SEG_LEN]]
            self.Xaug0 = Xaug[params.SEG_LEN]
            self.P0 = P_all[:,:,params.SEG_LEN]
            
        # If network output trajectory was not safe
        else:
            # Calculate remaining time for planning next trajectory segment
            remaining_planning_time = (self.seg_num+2)*params.T_SEG - (rospy.get_time() - self.init_time)
            # Init selected trajectory parameter dist
            selected_trajectory_param_dist_sq = np.inf
            # Sample new trajectory parameter if time remaining in current segment is sufficient
            print(remaining_planning_time)
            while remaining_planning_time > self.max_check_time:
                print("Resampling")
                # Sample new trajectory parameter near network output
                [kw, kv] = plan_util.sample_near_network_output(action_mean, action_cov)
                # Check safety of sampled trajectory parameter
                [isSafe, Xaug, Zaug, P_all, xnom_seg, unom_seg] = plan_util.check_trajectory_parameter_safety(kw, kv, self.x_nom0, self.Xaug0, self.P0, params.ENV_INFO)
                
                # Select trajectory if it is safe and if parameter distance is lower than previously selected parameter
                current_trajectory_param_dist_sq = (kw-kw0)**2 + (kv-kv0)**2
                if isSafe and current_trajectory_param_dist_sq < selected_trajectory_param_dist_sq:
                    safeTrajectoryFound = True
                    selected_Zaug = Zaug
                    #select current trajectory parameter
                    kw_safe = kw; kv_safe = kv
                    selected_trajectory_param_dist_sq = current_trajectory_param_dist_sq
                    selected_nominal_trajectory = [xnom_seg, unom_seg]
                    #store final nominal state, reachable set, state estimation covariance
                    self.x_nom0 = xnom_seg[:,[params.SEG_LEN]]
                    self.Xaug0 = Xaug[params.SEG_LEN]
                    self.P0 = P_all[:,:,params.SEG_LEN]
                    
                #calculate remaining time for planning upcoming segment
                remaining_planning_time = (self.seg_num+2)*params.T_SEG - (rospy.get_time() - self.init_time)

        if safeTrajectoryFound:
            print(" Generating trajectory segment with kw = ", round(kw_safe,2), "kv = ", round(kv_safe,2))
            print(" Segment endpoint: x = ", round(self.x_nom0[0][0],2), 
                                    " y = ", round(self.x_nom0[1][0],2), 
                                    " theta = ", round(self.x_nom0[2][0],2),"\n")
        
            # Publish trajectory
            self.traj_msg = NominalTrajectory()
            self.traj_msg.states = plan_util.wrap_states(xnom_seg)
            self.traj_msg.controls = plan_util.wrap_controls(unom_seg)
            self.traj_pub.publish(self.traj_msg)
        else:
            # Decide to execute fail-safe maneuver
            print("Failed to find safe trajectory - executing fail-safe maneuver")
            self.done = True

        # Check if we have reached the goal region
        reachedGoal = plan_util.is_trajectory_inside_region(xnom_seg, params.GOAL_ARR)
        if reachedGoal:
            rospy.loginfo("Reached goal")
            self.done = True

        self.seg_num += 1
        if self.seg_num >= params.MAX_SEGMENTS:
            rospy.loginfo("Max number of segments reached...stopping planning")
            self.done = True


    def run(self):
        """Run node

        """
        rospy.loginfo("Running Reachability Planner")

        while not rospy.is_shutdown() and not self.done:
            # connections = self.traj_pub.get_num_connections()
            # rospy.loginfo("Waiting for tracker, connections: %d", connections)

            # if connections > 0:
            
            # loop while replan is periodically called by Timer

            self.rate.sleep()

        rospy.loginfo("Exiting node")


if __name__ == '__main__':
    nnp = nn_planner()
    try:
        nnp.run()
    except rospy.ROSInterruptException:
        pass
