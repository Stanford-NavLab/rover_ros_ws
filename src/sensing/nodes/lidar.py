#!/usr/bin/env python

import rospy
import numpy as np
import time
import ros_numpy

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2

from planner.msg import State
import params.params as params

class Lidar():
    """Process LiDAR point cloud measurements

    """
    def __init__(self):
        # Parameters (eventually make these global)

        # Initialize node 
        rospy.init_node('Lidar', anonymous=True)
        self.rate = rospy.Rate(100)

        # Class variables
        self.height = None
        self.width = None

        # Publishers and subscribers
        pointcloud_sub = rospy.Subscriber('velodyne_points', PointCloud2, self.pointcloud_callback)


    def pointcloud_callback(self, data):
        """Point cloud subscriber callback

        Receive and save point cloud data.

        """
        self.height = data.height
        self.width = data.width

        #print(data.height)
        P = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)

        print("Points shape ", P.shape)
        #rospy.loginfo("Point cloud: (%f, %f, %f)", self.x, self.y, self.theta)



    def run(self):
        rospy.loginfo("Running Lidar node")
        while not rospy.is_shutdown():
            
            pass

            self.rate.sleep()
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    lidar = Lidar()
    try:
        lidar.run()
    except rospy.ROSInterruptException:
        pass