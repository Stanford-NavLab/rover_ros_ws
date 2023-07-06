#!/usr/bin/env python

from operator import ge
import rospy
import numpy as np
import ros_numpy

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2, PointCloud

from planner.msg import State
import params.params as params
from sensing.lidar_utils import detect_landmark, get_pos_measurement, rotate_points
from controller.controller_utils import wrap_angle


class LidarLMLocalization():
    """LiDAR-based landmark localization

    """
    def __init__(self):
        # Initialize node 
        rospy.init_node('lidar_landmark_localization', anonymous=True)
        self.rate = rospy.Rate(5)  # Same as controller rate

        # Class variables
        self.x_hat = None
        self.heading = None
        self.pos_measurement = None

        # Relative vector from robot to landmark in robot local frame
        # Initialize using robot initial state and landmark position
        self.landmark_relative_vec = rotate_points((params.LANDMARK_POS - params.X_0[:2].flatten())[None,:], params.X_0[2][0]).flatten()

        # Publishers
        self.pos_measurement_pub = rospy.Publisher('sensing/lidar/pos_measurement', Point, queue_size=10)
        ### For debugging
        self.pc_pub = rospy.Publisher('sensing/debug/pc', PointCloud, queue_size=10)

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
        #pos = data.pose.position
        #q = data.pose.orientation

        #quat = np.array([q.x, q.y, q.z, q.w])
        #r = R.from_quat(quat)
        #theta = wrap_angle(r.as_euler('zyx')[0])
        #self.x_hat = np.array([[pos.x],[pos.y],[theta],[0]])


    def pointcloud_callback(self, data):
        """Point cloud subscriber callback

        Receive point cloud data, detect the landmark, and use it to localize.

        """
        P = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)

        if self.x_hat is not None:
            self.landmark_relative_vec = detect_landmark(P, self.landmark_relative_vec)
            self.pos_measurement = get_pos_measurement(self.landmark_relative_vec, self.x_hat)
        else:
            print("Waiting for initial state estimate")

    
    def publish_measurement(self):
        """Publish position measurement
        
        """
        p = Point()
        p.x = self.pos_measurement[0]
        p.y = self.pos_measurement[1]
        self.pos_measurement_pub.publish(p)
        print(f'Published position measurement: ({p.x}, {p.y})')


    def run(self):
        rospy.loginfo("Running Lidar localization node")
        while not rospy.is_shutdown():
            
            if self.pos_measurement is not None:
                self.publish_measurement()

            self.rate.sleep()
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    lidar = LidarLMLocalization()
    try:
        lidar.run()
    except rospy.ROSInterruptException:
        pass
