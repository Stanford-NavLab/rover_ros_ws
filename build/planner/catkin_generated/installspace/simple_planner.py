#!/usr/bin/env python3

# Publishes a set nominal trajectory to be tracked

import rospy
import numpy as np
import sys

from planner.msg import State, Control, NominalTrajectory
import planner.planner_utils as plan_util
import params.params as params


class simple_planner():
    """Simple Planner

    Node which publishes a single nominal trajectory parameterized by desired linear and angular speed.

    """
    def __init__(self, kw, kv):
        # initialize node 
        rospy.init_node('simple_planner', anonymous=True, disable_signals=True)
        self.rate = rospy.Rate(10)

        # publishers
        self.traj_pub = rospy.Publisher('planner/traj', NominalTrajectory, queue_size=10)

        # subscribers
        mocap_sub = rospy.Subscriber('sensing/mocap', State, self.mocap_callback)

        # class variables
        self.x_0 = np.zeros((4,1))
        self.traj_msg = None
        self.kw = kw
        self.kv = kv

    
    def mocap_callback(self, data):
        """Mocap subscriber callback

        Save received initial state.

        """
        self.x_0 = np.array([[data.x],[data.y],[data.theta],[data.v]])


    def generate_trajectory(self):
        """Generate trajectory msg from parameters

        """
        rospy.loginfo("Generating nominal trajectory with w = %f, v = %f", kw, kv)

        x_nom, u_nom = plan_util.trajectory_parameter_to_nominal_trajectory(
            self.kw, self.kv, self.x_0, params.T_SEG, params.DT, params.MAX_ACC_MAG)

        rospy.loginfo("Desired final state: x = %f, y = %f, theta = %f", x_nom[0][-1], x_nom[1][-1], x_nom[2][-1])

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
        rospy.loginfo("Running Simple Planner")

        while not rospy.is_shutdown():
            connections = self.traj_pub.get_num_connections()
            rospy.loginfo("Connections: %d", connections)

            if connections > 0:
                self.generate_trajectory()
                self.publish_trajectory()
                rospy.loginfo("Published trajectory")
                rospy.loginfo("Exiting...")
                rospy.signal_shutdown("")
            self.rate.sleep()
            
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) < 3:
        # default kw and kv
        kw = 0.0
        kv = 0.5
    else:
        kw = float(sys.argv[1])
        kv = float(sys.argv[2])

    sp = simple_planner(kw, kv)
    try:
        sp.run()
    except rospy.ROSInterruptException:
        pass
