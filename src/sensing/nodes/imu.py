#!/usr/bin/env python

import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import QuaternionStamped
from std_msgs.msg import Float64

from controller.controller_utils import wrap_angle


class IMU_node():
    """Process and publish IMU measurements

    Currently orientation only.

    """
    def __init__(self):
        # Initialize node 
        rospy.init_node('imu_node', anonymous=True)
        self.rate = rospy.Rate(100)

        # Publishers and subscribers
        imu_sub = rospy.Subscriber('filter/quaternion', QuaternionStamped, self.imu_callback)
        self.imu_pub = rospy.Publisher('sensing/imu/heading', Float64, queue_size=1)


    def imu_callback(self, data):
        """IMU subscriber callback

        Republish IMU orientation as heading angle in radians.

        """
        q = data.quaternion

        quat = np.array([q.x, q.y, q.z, q.w])
        r = R.from_quat(quat)
        theta = wrap_angle(r.as_euler('zyx')[0])

        self.imu_pub.publish(theta)
        rospy.loginfo("Heading: %f", theta)


    def run(self):
        rospy.loginfo("Running IMU node")
        while not rospy.is_shutdown():

            self.rate.sleep()
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    imu = IMU_node()
    try:
        imu.run()
    except rospy.ROSInterruptException:
        pass