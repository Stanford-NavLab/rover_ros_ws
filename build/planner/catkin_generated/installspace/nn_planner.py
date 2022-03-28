#!/usr/bin/env python3

import numpy as np
import time
import rospy
import rospkg


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

        # Subscribers
        mocap_sub = rospy.Subscriber('sensing/mocap', State, self.mocap_callback)

        # Replan timer
        rospy.Timer(rospy.Duration(params.T_SEG), self.plan)
        self.init_time = rospy.get_time()
        print(self.init_time)

        # Class variables
        self.x_0 = np.zeros((4,1))
        self.traj_msg = None

        # Load learned model
        print("Loading model")
        model_file = rospkg.RosPack().get_path('planner') + '/models' + params.MODEL_NAME
        self.model = nn_util.load_model(model_file)

        self.done = False  # flag to check when to stop planning
        self.start_t = time.time() 
        print(self.start_t)

    
    def mocap_callback(self, data):
        """Mocap subscriber callback

        Save received initial state.

        """
        self.x_0 = np.array([[data.x],[data.y],[data.theta],[data.v]])


    def plan(self, event):
        """Plan next trajectory segment

        """
        # Get network output
        [action_mean, action_cov] = nn_util.evaluate_model(self.model, self.x_nom_RR[:,[-1]])
        kw = action_mean[0,0]; kv = action_mean[0,1]

        rospy.loginfo("Generating trajectory segment with kw = %f, kv= %f", kw, kv)

        # Generate trajectory
        x_nom, u_nom = plan_util.trajectory_parameter_to_nominal_trajectory(
            kw, kv, self.x_0, params.T_SEG, params.DT, params.MAX_ACC_MAG)

        rospy.loginfo("Desired final state: x = %f, y = %f, theta = %f", x_nom[0][-1], x_nom[1][-1], x_nom[2][-1])

        # Publish trajectory
        self.traj_msg = NominalTrajectory()
        self.traj_msg.states = plan_util.wrap_states(x_nom)
        self.traj_msg.controls = plan_util.wrap_controls(u_nom)
        self.traj_pub.publish(self.traj_msg)

        # Store final nominal state for planning next segment
        self.x_nom_end = x_nom[:,[params.SEG_LEN]]


    def run(self):
        """Run node

        """
        rospy.loginfo("Running Neural Network Planner")

        while not rospy.is_shutdown():
            # connections = self.traj_pub.get_num_connections()
            # rospy.loginfo("Waiting for tracker, connections: %d", connections)

            # if connections > 0:
            self.plan()

            self.rate.sleep()
            
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    nnp = nn_planner()
    try:
        nnp.run()
    except rospy.ROSInterruptException:
        pass
