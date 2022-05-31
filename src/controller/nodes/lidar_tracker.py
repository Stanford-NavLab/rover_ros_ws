#!/usr/bin/env python

import rospy
import numpy as np
import csv
import os

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point, Twist, PoseStamped
from std_msgs.msg import Float64

from planner.msg import State, Control, NominalTrajectory
from controller.controller_utils import wrap_angle, compute_control, lin_PWM, ang_PWM, EKF_prediction_step, EKF_correction_step
from planner.planner_utils import wrap_states
from planner.reachability_utils import generate_robot_matrices
import params.params as params

class lidar_tracker():
    """LiDAR Trajectory Tracker

    Tracks nominal trajectories with aid from landmark relative position
    measurements from LiDAR for state estimation.

    """
    def __init__(self):
        # Initialize node 
        rospy.init_node('lidar_tracker', anonymous=True)
        self.rate = rospy.Rate(1/params.DT)

        # Class variables
        self.idx = 0  # current index in the trajectory
        self.seg_num = 1  # current segment number
        self.X_nom_curr = None
        self.U_nom_curr = None
        self.X_nom_next = None
        self.U_nom_next = None

        self.x_hat = np.zeros((4,1))  # state estimate
        self.P = params.P_0  # covariance

        self.z = np.zeros((3,1))  # measurement
        self.z_gt = np.zeros((3,1))  # ground-truth measurement (no noise)
        self.imu_heading = 0  
        self.init_heading = None

        self.v_des = 0
        self.t_start = 0

        self.new_traj_flag = False

        # Publishers
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.state_est_pub = rospy.Publisher('controller/state_est', State, queue_size=1)

        # Subscribers
        traj_sub = rospy.Subscriber('planner/traj', NominalTrajectory, self.traj_callback)
        lidar_sub = rospy.Subscriber('sensing/lidar/pos_measurement', Point, self.lidar_callback)
        imu_sub = rospy.Subscriber('sensing/imu/heading', Float64, self.imu_callback)
        #mocap_sub = rospy.Subscriber('sensing/mocap', State, self.mocap_callback)
        mocap_sub = rospy.Subscriber('vrpn_client_node/rover/pose', PoseStamped, self.mocap_callback)

        # Logging
        path = '/home/navlab-nuc/Rover/flightroom_data/5_30_2022/tracking_logs'
        filename = 'lidar_track_'+str(rospy.get_time())+'.csv'
        self.logger = csv.writer(open(os.path.join(path, filename), 'w'))
        self.logger.writerow(['t', 'x', 'y', 'theta', 'z_x', 'z_y', 'z_theta', 
                              'x_nom', 'y_nom', 'theta_nom', 'v_nom', 
                              'x_hat', 'y_hat', 'theta_hat', 'v_hat', 'u_w', 'u_a'])


    def imu_callback(self, data):
        """IMU subscriber callback
        
        Save heading from IMU. Zero out at start.
        
        """
        if self.init_heading is None:
            self.init_heading = data.data
        self.imu_heading = wrap_angle(data.data - self.init_heading)


    def traj_callback(self, data):
        """Trajectory subscriber callback.

        Save nominal states and controls from received trajectory.

        """
        self.new_traj_flag = True
        # If no current trajectory yet, set it
        if self.X_nom_curr is None:
            self.X_nom_curr = data.states 
            self.U_nom_curr = data.controls 
            # Publish initial state estimate (for lidar node)
            self.state_est_pub.publish(data.states[0])
        # Otherwise, set next trajectory
        else:
            self.X_nom_next = data.states 
            self.U_nom_next = data.controls
        rospy.loginfo("Received trajectory of length %d", len(data.states))


    def mocap_callback(self, data):
        """Mocap subscriber callback.

        Save received ground-truth.

        """
        # Mocap data
        pos = data.pose.position
        q = data.pose.orientation

        quat = np.array([q.x, q.y, q.z, q.w])
        r = R.from_quat(quat)
        theta = wrap_angle(r.as_euler('zyx')[0])

        self.z_gt = np.array([pos.x, pos.y, theta])
        #print(self.z_gt)


    def lidar_callback(self, data):
        """Lidar subscriber callback

        Process lidar relative position measurement and run track.
        
        """
        # Form measurement z
        z = np.array([data.x, data.y, self.imu_heading])[:,None]  # (x, y, theta)
        
        if self.X_nom_curr is not None and not np.isnan(z[0]):
            # Call track
            self.track(z)


    def track(self, z):
        """Track next point in the current trajectory, and run the EKF for estimation.

        """
        print("idx ", self.idx, " ----------------------------------------")

        x_nom_msg = self.X_nom_curr[self.idx]
        x_nom = np.array([[x_nom_msg.x],[x_nom_msg.y],[x_nom_msg.theta],[x_nom_msg.v]])
        u_nom_msg = self.U_nom_curr[self.idx]
        u_nom = np.array([[u_nom_msg.omega],[u_nom_msg.a]])
        
        # Start of first segment
        if self.idx == 0 and self.seg_num == 1:
            self.x_hat = x_nom

        A,B,C,K = generate_robot_matrices(x_nom, u_nom, params.Q_LQR, params.R_LQR, params.DT)
        # K = np.array([[0, 0, 1, 0],
        #               [0, 0, 0, 1]])

        # ======== EKF Update ========
        self.x_hat, self.P = EKF_correction_step(self.x_hat, self.P, z, C, params.R_EKF)
        self.state_est_pub.publish(wrap_states(self.x_hat)[0])

        # ======== Apply feedback control law ========
        u = compute_control(x_nom, u_nom, self.x_hat, K)

        # Create motor command msg
        motor_cmd = Twist()
        
        # Closed-loop
        self.v_des += params.DT * u[1][0]  # integrate acceleration
        motor_cmd.linear.x = lin_PWM(self.v_des, u[0][0])
        motor_cmd.angular.z = ang_PWM(self.v_des, u[0][0])

        print(" - v_des: ", round(self.v_des,2), " u_a: ", round(u[1][0],2), " u_w: ", round(u[0][0],2))
        print(" - lin PWM: ", round(motor_cmd.linear.x,2), ", ang PWM: ", round(motor_cmd.angular.z,2))

        self.cmd_pub.publish(motor_cmd)

        self.idx += 1

        # Log data TODO: update this
        self.logger.writerow([rospy.get_time(), self.z_gt[0], self.z_gt[1], self.z_gt[2], 
                              z[0][0], z[1][0], z[2][0],
                              x_nom[0][0], x_nom[1][0], x_nom[2][0], x_nom[3][0], self.x_hat[0][0], 
                              self.x_hat[1][0], self.x_hat[2][0], self.x_hat[3][0], u[0][0], u[1][0]])

        # ======== Check for end of trajectory ========
        if self.idx >= params.SEG_LEN:
            # Finished tracking current trajectory 
            rospy.loginfo("Finished tracking trajectory")

            # If we have a next trajectory, switch to tracking that. Otherwise, continue the trajectory (braking maneuver)
            if self.X_nom_next is not None:
                rospy.loginfo("Switching to next trajectory")
                self.X_nom_curr = self.X_nom_next
                self.U_nom_curr = self.U_nom_next
                self.X_nom_next = None
                self.X_nom_next = None
                self.idx = 0
                self.seg_num += 1
            else:
                rospy.loginfo("Executing braking maneuver")

        # ======== Check for end of braking maneuver ========
        if self.idx >= len(self.U_nom_curr):
            rospy.loginfo("Braking maneuver completed")
            # Reset class variables
            self.X_nom_curr = None
            self.U_nom_curr = None
            self.v_des = 0
            self.idx = 0

            # Send multiple stop commands in case some don't go through
            for i in range(5):
                self.rate.sleep()
                self.stop_motors()

        # ======== EKF Predict ========
        self.x_hat, self.P = EKF_prediction_step(self.x_hat, u, self.P, A, params.Q_EKF, params.DT)

    
    def stop_motors(self):
        """Send stop command to all motors

        """
        rospy.loginfo("Stopping motors")
        motor_cmd = Twist()
        motor_cmd.linear.x = 0.0
        motor_cmd.angular.z = 0.0
        self.cmd_pub.publish(motor_cmd)


    def run(self):
        """Run node

        """
        rospy.loginfo("Running Lidar Tracker")
        while not rospy.is_shutdown():

            # Loop while track is called from lidar_callback
            self.rate.sleep()
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    lt = lidar_tracker()
    try:
        lt.run()
    except rospy.ROSInterruptException:
        pass
