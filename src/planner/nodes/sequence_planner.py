#!/usr/bin/env python

# Publishes a sequence of nominal trajectories to be tracked

import rospy
import numpy as np
import sys

from planner.msg import State, Control, NominalTrajectory
import planner.planner_utils as plan_util
import params.params as params


class sequence_planner():
    """Sequence Planner

    Node which publishes a sequence of nominal trajectories in succession.

    Attributes
    ----------
    TODO

    Methods
    -------
    TODO

    """
    def __init__(self):
        # parameters (eventually make global)
        self.max_acc_mag = 0.25
        self.t_plan = 3.0  # time duration of each trajectory
        self.t_next = 3.0  # time between sending trajectories
        self.dt = 0.2

        # initialize node 
        rospy.init_node('sequence_planner', anonymous=True, disable_signals=True)
        self.rate = rospy.Rate(10)

        # publishers
        self.traj_pub = rospy.Publisher('planner/traj', NominalTrajectory, queue_size=10)

        self.x_nom_end = params.X_0
        self.traj_msg = None

        # trajectory sequence
        self.k_seq = np.array([[0.27, -0.19, -0.03, 0.01, -0.09, -0.12, 0.07],
                               [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]])
        self.n_traj = self.k_seq.shape[1]


    def generate_trajectory(self, kw, kv):
        """Generate trajectory msg from parameters

        """
        rospy.loginfo("Generating nominal trajectory with w = %f, v = %f", kw, kv)

        x_nom, u_nom = plan_util.trajectory_parameter_to_nominal_trajectory(
            kw, kv, self.x_nom_end, params.T_SEG, params.DT, params.MAX_ACC_MAG)

        rospy.loginfo("Final state: x = %f, y = %f, theta = %f", x_nom[0][-1], x_nom[1][-1], x_nom[2][-1])

        self.x_nom_end = x_nom[:,[params.SEG_LEN]]

        self.traj_msg = NominalTrajectory()
        self.traj_msg.states = plan_util.wrap_states(x_nom)
        self.traj_msg.controls = plan_util.wrap_controls(u_nom)
        

    def publish_trajectory(self):
        """Publish trajectory message

        """
        self.traj_pub.publish(self.traj_msg)


    def run(self):
        """Run node

        """
        rospy.loginfo("Running Sequence Planner")

        while not rospy.is_shutdown():
            connections = self.traj_pub.get_num_connections()
            rospy.loginfo("Connections: %d", connections)

            if connections > 0:
                for i in range(self.n_traj):
                    self.generate_trajectory(self.k_seq[0][i], self.k_seq[1][i])
                    self.publish_trajectory()
                    rospy.loginfo("Published trajectory segment")
                    rospy.sleep(self.t_next)
                rospy.loginfo("Exiting...")
                rospy.signal_shutdown("")
            self.rate.sleep()
            
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    sp = sequence_planner()
    try:
        sp.run()
    except rospy.ROSInterruptException:
        pass
