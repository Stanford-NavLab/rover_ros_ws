#!/usr/bin/env python

import rospy
import numpy as np
import rospkg

from planner.msg import State, Control, NominalTrajectory
import planner.planner_utils as plan_util
import planner.reachability_utils as reach_util
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

        # Subscribers
        mocap_sub = rospy.Subscriber('sensing/mocap', State, self.mocap_callback)

        # Replan timer
        rospy.Timer(rospy.Duration(params.T_SEG), self.replan)
        self.init_time = rospy.get_time()

        # Class variables
        self.x_0 = np.zeros((4,1))
        self.traj_msg = None

        # Load learned model
        print("Loading model")
        model_file = rospkg.RosPack().get_path('planner') + '/models/' + params.MODEL_NAME
        self.model = nn_util.load_model(model_file)

        # Get initial reach set
        Xaug0 = reach_util.initialize_reachability_analysis(params.X_0, params.P_0)

        # Calibration to estimate max time it would take to sample a trajectory parameter and check its safety
        self.max_check_time = plan_util.calibrate_sample_safety_check_time(
            params.X_0, Xaug0, params.P_0, env_info, params)


    def mocap_callback(self, data):
        """Mocap subscriber callback

        Save received initial state.

        """
        self.x_0 = np.array([[data.x],[data.y],[data.theta],[data.v]])
        

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


    def run(self):
        """Run node

        """
        rospy.loginfo("Running Neural Network Planner")

        while not rospy.is_shutdown():
            connections = self.traj_pub.get_num_connections()
            rospy.loginfo("Connections: %d", connections)

            if connections > 0:
                self.generate_trajectory()
                self.publish_trajectory()
                rospy.loginfo("Published trajectory")
                rospy.signal_shutdown("Exiting...")
            self.rate.sleep()
            
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    nnp = nn_planner()
    try:
        nnp.run()
    except rospy.ROSInterruptException:
        pass
