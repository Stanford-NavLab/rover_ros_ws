#!/usr/bin/env python

# Publishes a set nominal trajectory to be tracked

import rospy
import numpy as np
import sys

from geometry_msgs.msg import Twist

from planner.msg import State, Control, NominalTrajectory
import planner.planner_utils as plan_util
import params.params as params


class OpenLoopPlanner():
    """Simple Planner

    Node which publishes a single nominal trajectory parameterized by desired linear and angular speed.

    """
    def __init__(self):
        # initialize node 
        rospy.init_node('open_loop_planner', anonymous=True, disable_signals=True)
        self.rate = rospy.Rate(10)

        # publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # subscribers
        #mocap_sub = rospy.Subscriber('sensing/mocap', State, self.mocap_callback)

    
    # def mocap_callback(self, data):
    #     """Mocap subscriber callback

    #     Save received initial state.

    #     """
    #     self.x_0 = np.array([[data.x],[data.y],[data.theta],[data.v]])


    def send_cmd(self, linear_x, linear_y, angular_z):
        """Generate trajectory msg from parameters

        """
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = linear_y
        twist_msg.angular.z = angular_z
        print(linear_x, linear_y, angular_z)
        self.cmd_pub.publish(twist_msg)


    def turn_90(self, ccw=True):
        if ccw is True:
            self.send_cmd(0.0, 0.0, 0.2)
        else:
            self.send_cmd(0.0, 0.0, -0.2)
        rospy.sleep(2.15)


    def drive_straight(self, num_tiles):
        self.send_cmd(0.2, 0.0, 0.0)
        rospy.sleep(num_tiles*2)


    def stop(self):
        self.send_cmd(0.0, 0.0, 0.0)
        

    def send_trajectory(self):
        """Publish trajectory message

        """
        rospy.sleep(1)
        self.drive_straight(2)
        self.turn_90()
        self.drive_straight(2)
        self.turn_90(False)
        self.drive_straight(2)
        self.turn_90(False)
        self.drive_straight(1)
        self.turn_90()
        self.drive_straight(4)
        self.stop()



    def run(self):
        """Run node

        """
        rospy.loginfo("Running Simple Planner")

        # while not rospy.is_shutdown():
            
        #     self.rate.sleep()
        self.send_trajectory()

        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()


if __name__ == '__main__':

    olp = OpenLoopPlanner()
    try:
        olp.run()
    except rospy.ROSInterruptException:
        pass
