#!/usr/bin/env python

from operator import ge
import rospy
import numpy as np
import ros_numpy
import time

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PolygonStamped, Point32
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2, PointCloud

from planner.msg import State
import params.params as params
from sensing.lidar_utils import detect_landmark, get_pos_measurement, rotate_points
from controller.controller_utils import wrap_angle

from planeslam.scan import pc_to_scan


class LidarPlaneSLAM():
    """LiDAR Plane-based SLAM

    """
    def __init__(self):
        # Initialize node 
        rospy.init_node('plane_slam', anonymous=True)
        self.rate = rospy.Rate(5)  

        # Publishers
        self.plane_pub = rospy.Publisher('sensing/planes', PolygonStamped, queue_size=10)

        # Subscribers
        pointcloud_sub = rospy.Subscriber('velodyne_points', PointCloud2, self.pointcloud_callback)
        imu_sub = rospy.Subscriber('sensing/imu/heading', Float64, self.imu_callback)
        state_est_sub = rospy.Subscriber('controller/state_est', State, self.state_est_callback)
        #state_est_sub = rospy.Subscriber('vrpn_client_node/rover/pose', PoseStamped, self.state_est_callback)

        self.path = '/home/navlab-nuc/Rover/lidar_data/5_15_2022/fr_config_5'
        self.frame_num = 0


    def imu_callback(self, data):
        """IMU callback
        
        Store heading.

        """
        self.heading = data.data

    
    def state_est_callback(self, data):
        """State estimate callback
        
        Store state estimate.

        """
        self.x_hat = np.array([[data.x],[data.y],[data.theta],[data.v]])


    def pointcloud_callback(self, data):
        """Point cloud subscriber callback

        Receive point cloud data, detect the landmark, and use it to localize.

        """
        P = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
        start_time = time.time()
        scan = pc_to_scan(P)
        print(time.time() - start_time)

    
    def publish_planes(self):
        """
        """
        poly = PolygonStamped()
        
        p1 = Point32()
        p1.x = 1
        p1.y = 1
        p1.z = 0

        p2 = Point32()
        p2.x = 1
        p2.y = -1
        p2.z = 0

        p3 = Point32()
        p3.x = -1
        p3.y = -1
        p3.z = 0

        p4 = Point32()
        p4.x = -1
        p4.y = 1
        p4.z = 0

        poly.polygon.points = [p1, p2, p3, p4]

        self.plane_pub.publish(poly)


    def run(self):
        rospy.loginfo("Running Plane SLAM")
        while not rospy.is_shutdown():

            self.publish_planes()
            self.rate.sleep()
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    lidar = LidarPlaneSLAM()
    try:
        lidar.run()
    except rospy.ROSInterruptException:
        pass
