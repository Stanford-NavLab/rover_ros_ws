#!/usr/bin/env python

import rospy
import numpy as np
import os
import ros_numpy

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped


class LidarDC():
    """Collect LiDAR point cloud measurements

    """
    def __init__(self):
        # Initialize node 
        rospy.init_node('Lidar', anonymous=True)
        self.rate = rospy.Rate(5)  # Same as controller rate

        self.pose = None

        # Subscribers
        pointcloud_sub = rospy.Subscriber('velodyne_points', PointCloud2, self.pointcloud_callback)
        vrpn_sub = rospy.Subscriber('vrpn_client_node/rover/pose', PoseStamped, self.vrpn_callback)

        self.path = '/home/navlab-nuc/Rover/lidar_data/9_19_2022/flightroom/run_3'
        self.frame_num = 0


    def vrpn_callback(self, data):
        """Mocap subscriber callback

        Receive and save mocap data as measurement.

        """
        pos = data.pose.position
        q = data.pose.orientation

        self.pose = np.array([pos.x, pos.y, pos.z, q.x, q.y, q.z, q.w])


    def pointcloud_callback(self, data):
        """Point cloud subscriber callback

        Receive and save point cloud data.

        Currently saved unorganized point clouds.

        """
        P = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)

        # Save points as .npy
        filename = 'pc_'+str(self.frame_num)+'.npy'
        np.save(os.path.join(self.path, 'pcs', filename), P)
        print("Saved point cloud ", self.frame_num)

        # Save pose as .npy
        filename = 'pose_'+str(self.frame_num)+'.npy'
        np.save(os.path.join(self.path, 'poses', filename), self.pose)
        self.frame_num += 1


    def run(self):
        rospy.loginfo("Running Lidar data collection node")
        while not rospy.is_shutdown():

            self.rate.sleep()
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    lidardc = LidarDC()
    try:
        lidardc.run()
    except rospy.ROSInterruptException:
        pass
