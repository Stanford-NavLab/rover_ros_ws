#!/usr/bin/env python

import numpy as np
import time
import rospy
import rospkg
import csv
import os


from planner.msg import State, NominalTrajectory
import planner.planner_utils as plan_util
import planner.NN_utils as nn_util
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
        self.x_nom_end = params.X_0
        #self.x_nom_hist = params.X_0
        self.traj_msg = None

        # Load learned model
        rospy.loginfo("Loading model")
        model_file = rospkg.RosPack().get_path('planner') + '/models/' + params.MODEL_NAME
        self.model = nn_util.load_model(model_file)

        self.done = False  # flag to check when to stop planning
        self.seg_num = 1  # current segment number

        # Logging
        path = '/home/navlab-nuc/Rover/flightroom_data/6_15_2022/planner_logs'
        filename = 'nn_plan_'+str(rospy.get_time())+'.csv'
        self.log_file = open(os.path.join(path, filename), 'w')
        self.logger = csv.writer(self.log_file)
        self.logger.writerow(['t', 'kw', 'kv'])


    def replan(self, event):
        """Plan next trajectory segment

        """
        print("Replanning: t = ", rospy.get_time() - self.init_time)

        # Get network output
        [action_mean, action_cov] = nn_util.evaluate_model(self.model, self.x_nom_end)
        kw = action_mean[0,0]; kv = action_mean[0,1]

        print(" Generating trajectory segment with kw = ", round(kw,2), "kv = ", round(kv,2))

        # Generate trajectory
        x_nom, u_nom = plan_util.trajectory_parameter_to_nominal_trajectory(
            kw, kv, self.x_nom_end, params.T_SEG, params.DT, params.MAX_ACC_MAG)

        # Publish trajectory
        self.traj_msg = NominalTrajectory()
        self.traj_msg.states = plan_util.wrap_states(x_nom)
        self.traj_msg.controls = plan_util.wrap_controls(u_nom)
        self.traj_pub.publish(self.traj_msg)

        # Store final nominal state for planning next segment
        self.x_nom_end = x_nom[:,[params.SEG_LEN]]

        print(" Segment endpoint: x = ", round(self.x_nom_end[0][0],2), 
                                " y = ", round(self.x_nom_end[1][0],2), 
                                " theta = ", round(self.x_nom_end[2][0],2),"\n")
        
        # Check if we have reached the goal region
        reachedGoal = plan_util.is_trajectory_inside_region(x_nom, params.GOAL_ARR)
        if reachedGoal:
            rospy.loginfo("Reached goal")
            self.done = True

        self.seg_num += 1
        if self.seg_num >= params.MAX_SEGMENTS:
            rospy.loginfo("Max number of segments reached...stopping planning")
            self.done = True

        # Write to log
        self.logger.writerow([rospy.get_time(), kw, kv])


    def run(self):
        """Run node

        """
        rospy.loginfo("Running Neural Network Planner")

        while not rospy.is_shutdown() and not self.done:
            # connections = self.traj_pub.get_num_connections()
            # rospy.loginfo("Waiting for tracker, connections: %d", connections)

            # if connections > 0:
            
            # loop while replan is periodically called by Timer

            self.rate.sleep()

        self.log_file.close()
        rospy.loginfo("Exiting node")


if __name__ == '__main__':
    nnp = nn_planner()
    try:
        nnp.run()
    except rospy.ROSInterruptException:
        pass
