#!/usr/bin/env python3

import rospy
import numpy as np
import time

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped

from planner.msg import State, Control, NominalTrajectory
from planner.planner_utils import generate_robot_matrices
from controller.controller_utils import wrap_angle, EKF_prediction_step, EKF_correction_step

class EKF():
    """Extended Kalman Filter for state estimation

    """
    def __init__(self):
        # parameters (eventually make these global)
        # self.dt = 0.2
        # self.t_plan = 3.0
        # self.Q_lqr = np.diag(np.array([5, 5, 10, 100]))
        # self.R_lqr = np.diag(np.array([10, 1]))

        # initialize node 
        rospy.init_node('EKF', anonymous=True)
        self.rate = rospy.Rate(10)

        # class variables
        self.x_hat = None  # [x, y, theta, v]
        self.t_prev = 0
        self.z = None

        # publishers and subscribers
        mocap_sub = rospy.Subscriber('vrpn_client_node/rover/pose', PoseStamped, self.mocap_callback)
        self.state_est_pub = rospy.Publisher('controller/state_est', State, queue_size=1)


    def mocap_callback(self, data):
        """Mocap subscriber callback

        Receive and save mocap data as measurement.

        """
        pos = data.pose.position
        q = data.pose.orientation

        quat = np.array([q.x, q.y, q.z, q.w])
        r = R.from_quat(quat)
        self.x_hat[2] = wrap_angle(r.as_euler('zyx')[0])

        dt = time.time() - self.t_prev
        self.v = np.sqrt((self.x - pos.x)**2 + (self.y - pos.y)**2) / dt
        self.t_prev = time.time()

        self.x = pos.x; self.y = pos.y

        rospy.loginfo("Received data: (%f, %f, %f, %f)", self.x, self.y, self.theta, self.v)


    def publish_state_est(self):
        """Publish current state estimate

        """
        s = State()
        s.x = self.x_hat[0]
        s.y = self.x_hat[1]
        s.theta = self.x_hat[2] 
        s.v = self.x_hat[3]
        self.state_est_pub.publish(s)


    def predict(self):
        """Predict step 

        ---.

        """
        pass


    def update(self):
        """Predict step 

        ---.

        """
        pass


    def run(self):
        rospy.loginfo("Running EKF")
        while not rospy.is_shutdown():
            
            self.predict()
            self.update()
            self.publish_state_est()

            self.rate.sleep()
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    ekf = EKF()
    try:
        ekf.run()
    except rospy.ROSInterruptException:
        pass