#!/usr/bin/env python

import rospy
import numpy as np
import time

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

from planner.msg import State
from controller.controller_utils import wrap_angle
import params.params as params

class Mocap():
    """Process and publish motion capture measurements

    """
    def __init__(self):
        # Initialize node 
        rospy.init_node('mocap', anonymous=True)
        self.rate = rospy.Rate(100)

        # Class variables
        self.x = 0
        self.y = 0
        self.theta = 0
        self.x_std = np.sqrt(params.R_EKF[0,0])
        self.y_std = np.sqrt(params.R_EKF[1,1])
        self.theta_std = np.sqrt(params.R_EKF[2,2])

        # Publishers and subscribers
        vrpn_sub = rospy.Subscriber('vrpn_client_node/rover/pose', PoseStamped, self.vrpn_callback)
        self.mocap_pub = rospy.Publisher('sensing/mocap', State, queue_size=1)
        self.mocap_noisy_pub = rospy.Publisher('sensing/mocap_noisy', State, queue_size=1)


    def vrpn_callback(self, data):
        """Mocap subscriber callback

        Receive and save mocap data as measurement.

        """
        pos = data.pose.position
        q = data.pose.orientation

        quat = np.array([q.x, q.y, q.z, q.w])
        r = R.from_quat(quat)
        self.theta = wrap_angle(r.as_euler('zyx')[0])

        self.x = pos.x; self.y = pos.y

        rospy.loginfo("Received data: (%f, %f, %f)", self.x, self.y, self.theta)


    def publish(self):
        """Publish latest ground-truth and noisy measurements

        """
        s = State()
        s.x = self.x
        s.y = self.y
        s.theta = self.theta
        s.v = 0.0  # no velocity measurement
        self.mocap_pub.publish(s)

        s = State()
        s.x = self.x + np.random.normal(0, self.x_std)
        s.y = self.y + np.random.normal(0, self.y_std)
        s.theta = self.theta + np.random.normal(0, self.theta_std)
        s.v = 0.0  # no velocity measurement
        self.mocap_noisy_pub.publish(s)


    def run(self):
        rospy.loginfo("Running Mocap node")
        while not rospy.is_shutdown():
            
            self.publish()

            self.rate.sleep()
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    mocap = Mocap()
    try:
        mocap.run()
    except rospy.ROSInterruptException:
        pass