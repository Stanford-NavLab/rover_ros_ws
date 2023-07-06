#!/usr/bin/env python

import rospy
import numpy as np
import csv
import os

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64, Bool

from planner.msg import State, Control, NominalTrajectory
from controller.controller_utils import compute_control, lin_PWM, ang_PWM, EKF_prediction_step, EKF_correction_step
from planner.planner_utils import wrap_states
from planner.reachability_utils import generate_robot_matrices
import params.params as params


class traj_tracker():
    """Trajectory tracker

    Tracks nominal trajectories by applying linear control feedback using 
    state estimate and sending motor commands. If a new trajectory is received 
    while tracking the current trajectory, it will switch to tracking the new
    trajectory once it has finished the current trajectory.

    """
    def __init__(self):
        # Initialize node 
        rospy.init_node('traj_tracker', anonymous=True)
        self.rate = rospy.Rate(1/params.DT)

        # Class variables
        self.idx = 0  # current index in the trajectory
        self.X_nom_curr = None
        self.U_nom_curr = None

        self.x_hat = np.zeros((4,1))  # state estimate
        self.P = params.P_0  # covariance

        self.z = np.zeros((3,1))  # measurement
        self.z_gt = np.zeros((3,1))  # ground-truth measurement (no noise)
        self.v_des = 0
        self.t_start = 0

        self.traj_len = 0

        # Publishers
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.state_est_pub = rospy.Publisher('controller/state_est', State, queue_size=1)
        self.debug_pub = rospy.Publisher('debug/x_err', Float64, queue_size=1)
        # TODO: "done" publisher - tell planner when done so can reset and send next trajectory
        self.done_pub = rospy.Publisher('controller/done', Bool, queue_size=1)

        # Subscribers
        traj_sub = rospy.Subscriber('planner/traj', NominalTrajectory, self.traj_callback)
        measurement_sub = rospy.Subscriber('sensing/mocap_noisy', State, self.measurement_callback)
        gt_sub = rospy.Subscriber('sensing/mocap', State, self.gt_callback)

        # Logging
        path = '/home/navlab-nuc/Rover/flightroom_data/4_12_2022/debug/'
        filename = 'track_'+str(rospy.get_time())+'.csv'
        self.logger = csv.writer(open(os.path.join(path, filename), 'w'))
        self.logger.writerow(['t', 'x', 'y', 'theta', 'z_x', 'z_y', 'z_theta', 
                              'x_nom', 'y_nom', 'theta_nom', 'v_nom', 
                              'x_hat', 'y_hat', 'theta_hat', 'v_hat', 'u_w', 'u_a'])


    def traj_callback(self, data):
        """Trajectory subscriber callback.

        Save nominal states and controls from received trajectory.

        """
        self.X_nom_curr = data.states 
        self.U_nom_curr = data.controls 
        rospy.loginfo("Received trajectory of length %d", len(data.states))
        self.traj_len = len(data.states)
    

    def measurement_callback(self, data):
        """Measurement subscriber callback.

        Save received measurement.

        """
        # For mocap measurement
        self.z = np.array([[data.x],[data.y],[data.theta]])


    def gt_callback(self, data):
        """Ground-truth subscriber callback.

        Save received ground-truth.

        """
        # Mocap data
        self.z_gt = np.array([[data.x],[data.y],[data.theta]])


    def track(self):
        """Track next point in the current trajectory, and run the EKF for estimation.

        TODO

        """
        print("idx ", self.idx, " ----------------------------------------")

        x_nom_msg = self.X_nom_curr[self.idx]
        x_nom = np.array([[x_nom_msg.x],[x_nom_msg.y],[x_nom_msg.theta],[x_nom_msg.v]])
        u_nom_msg = self.U_nom_curr[self.idx]
        u_nom = np.array([[u_nom_msg.omega],[u_nom_msg.a]])
        
        # Start of trajectory
        if self.idx == 0:
            self.x_hat = x_nom

        A,B,C,K = generate_robot_matrices(x_nom, u_nom, params.Q_LQR, params.R_LQR, params.DT)

        # ======== EKF Update ========
        self.x_hat, self.P = EKF_correction_step(self.x_hat, self.P, self.z_gt, C, params.R_EKF)
        self.state_est_pub.publish(wrap_states(self.x_hat)[0])

        # ======== Apply feedback control law ========
        u = compute_control(x_nom, u_nom, self.x_hat, K)

        # Create motor command msg
        motor_cmd = Twist()
        
        # Closed-loop
        self.v_des += params.DT * u[1][0]  # integrate acceleration
        # motor_cmd.linear.x = lin_PWM(self.v_des, u[0][0])
        motor_cmd.linear.x = 0.0
        motor_cmd.angular.z = ang_PWM(self.v_des, u[0][0])

        print(" - v_des: ", round(self.v_des,2), " u_a: ", round(u[1][0],2), " u_w: ", round(u[0][0],2))
        print(" - lin PWM: ", round(motor_cmd.linear.x,2), ", ang PWM: ", round(motor_cmd.angular.z,2))

        self.cmd_pub.publish(motor_cmd)

        self.idx += 1

        # Log data
        self.logger.writerow([rospy.get_time(), self.z_gt[0][0], self.z_gt[1][0], self.z_gt[2][0], 
                              self.z[0][0], self.z[1][0], self.z[2][0],
                              x_nom[0][0], x_nom[1][0], x_nom[2][0], x_nom[3][0], self.x_hat[0][0], 
                              self.x_hat[1][0], self.x_hat[2][0], self.x_hat[3][0], u[0][0], u[1][0]])
        
        # if self.idx == int(self.traj_len/2):
        #     self.v_des = 0
        

        # ======== Check for end of trajectory ========
        if self.idx >= self.traj_len:
            # Finished tracking current trajectory 
            rospy.loginfo("Finished tracking trajectory")

            # Reset class variables
            self.X_nom_curr = None
            self.U_nom_curr = None
            self.v_des = 0
            self.idx = 0

            # Send multiple stop commands in case some don't go through
            for i in range(5):
                self.rate.sleep()
                self.stop_motors()

            self.done_pub.publish(True)
            

        # ======== EKF Predict ========
        self.x_hat, self.P = EKF_prediction_step(self.x_hat, u, self.P, A, params.Q_EKF, params.DT)

        # ======== Debugging ========
        x_err = self.z[0] - x_nom[0]
        self.debug_pub.publish(x_err)

    
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
        rospy.loginfo("Running Trajectory Tracker")
        while not rospy.is_shutdown():
            
            if self.X_nom_curr is not None:
                self.track()

            self.rate.sleep()
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    tt = traj_tracker()
    try:
        tt.run()
    except rospy.ROSInterruptException:
        pass
