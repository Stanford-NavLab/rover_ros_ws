#!/usr/bin/env python

import rospy
import numpy as np
import time
import cv2
import csv
import os
import sys
from cv_bridge import CvBridge, CvBridgeError

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import Image

from planner.msg import State
from controller.controller_utils import wrap_angle
import params.params as params

class CameraLogger():
    """Process and publish motion capture measurements

    """
    def __init__(self):
        # Initialize node 
        rospy.init_node('cam_logger', disable_signals=True, anonymous=True)
        self.rate = rospy.Rate(10)

        # Class variables
        self.pose = np.zeros(7)
        self.frame_num = 0
        
        self.bridge = CvBridge()
        self.img_data = None

        # Logging
        self.path = '/home/navlab-nuc/Rover/nerf_data/10_25_2022/run_1/'
        filename = 'poses.csv'
        self.file = open(os.path.join(self.path, filename), 'w')
        self.pose_logger = csv.writer(self.file)
        self.pose_logger.writerow(['frame', 't', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

        # Publishers and subscribers
        vrpn_sub = rospy.Subscriber('vrpn_client_node/rover/pose', PoseStamped, self.vrpn_callback)
        cam_sub = rospy.Subscriber('cv_camera/image_raw', Image, self.cam_callback)


    def cam_callback(self, data):
        #print("Received an image!")
        self.img_data = data

    
    def save_img(self):
        print("Saving image ", self.frame_num)
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(self.img_data, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg 
            timestamp = rospy.get_time()
            cv2.imwrite(os.path.join(self.path, 'images', str(self.frame_num)+'.jpeg'), cv2_img)
        
            # Log pose
            self.pose_logger.writerow([self.frame_num, rospy.get_time(), self.pose[0], self.pose[1], self.pose[2],
                self.pose[3], self.pose[4], self.pose[5], self.pose[6]])
            self.frame_num += 1


    def vrpn_callback(self, data):
        """Mocap subscriber callback

        Receive and save mocap data as measurement.

        """
        #print("Received mocap data")
        pos = data.pose.position
        q = data.pose.orientation

        self.pose = np.array([pos.x, pos.y, pos.z, q.x, q.y, q.z, q.w])


    def run(self):
        rospy.loginfo("Running Camera Logger node")
        while not rospy.is_shutdown():
            
            try:
                input()
                self.save_img()
                self.rate.sleep()
            except KeyboardInterrupt:
                print("break")
                self.file.close()
                sys.exit()
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    CL = CameraLogger()
    try:
        CL.run()
    except rospy.ROSInterruptException:
        pass