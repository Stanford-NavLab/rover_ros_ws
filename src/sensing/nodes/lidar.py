#!/usr/bin/env python

import rospy
import numpy as np
import time
import os
import ros_numpy

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2

from planner.msg import State
import params.params as params
from sensing.lidar_utils import detect_landmark


class Lidar():
    """Process LiDAR point cloud measurements

    """
    def __init__(self):
        # Parameters (eventually make these global)

        # Initialize node 
        rospy.init_node('Lidar', anonymous=True)
        self.rate = rospy.Rate(5)  # Same as controller rate

        # Class variables
        self.heading = None
        self.landmark_pos = None

        # Publishers and subscribers
        pointcloud_sub = rospy.Subscriber('velodyne_points', PointCloud2, self.pointcloud_callback)
        imu_sub = rospy.Subscribe('sensing/imu/heading', Float64, self.imu_callback)
        pos_measurement_pub = rospy.Publisher('sensing/lidar/pos_measurement', Point, queue_size=10)

        self.path = '/home/navlab-nuc/Rover/lidar_data/5_15_2022/fr_config_5'
        self.frame_num = 0


    def imu_callback(self, data):
        """IMU callback
        
        Store heading.

        """
        self.heading = data.data


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

        # Save points as .npy
        filename = 'pc_'+str(self.frame_num)+'.npy'
        np.save(os.path.join(self.path, filename), P)
        self.frame_num += 1

    
    def publish_measurement(self):
        """Publish measurement
        
        """
        p = Point()
        p.x = self.landmark_pos[0]
        p.y = self.landmark_pos[1]
        self.pos_measurement_pub.publish(p)


    def run(self):
        rospy.loginfo("Running Lidar node")
        while not rospy.is_shutdown():
            
            self.publish_detection()

            self.rate.sleep()
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    lidar = Lidar()
    try:
        lidar.run()
    except rospy.ROSInterruptException:
        pass
