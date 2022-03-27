#!/usr/bin/env python3

import rospy
import numpy as np
import time

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64

from planner.msg import State, Control, NominalTrajectory
from controller.controller_utils import compute_control, v_to_PWM, omega_to_PWM, EKF_prediction_step, EKF_correction_step
from planner.planner_utils import generate_robot_matrices, wrap_states

class traj_tracker():
    """Trajectory tracker

    Tracks nominal trajectories by applying linear control feedback using 
    state estimate and sending motor commands. If a new trajectory is received 
    while tracking the current trajectory, it will switch to tracking the new
    trajectory once it has finished the current trajectory.

    """
    def __init__(self):
        # Parameters (eventually make these global)
        self.dt = 0.2
        self.t_plan = 3.0  # trajectory time length (excluding braking maneuver)
        self.Q_lqr = np.diag([5, 5, 10, 100])
        self.R_lqr = np.diag([100, 100])
        self.Q_ekf = np.diag([0.0001, 0.0001, 0.0005, 0.0001])
        self.R_ekf = np.diag([0.1, 0.1, 0.001, 0.01])

        # Initialize node 
        rospy.init_node('traj_tracker', anonymous=True)
        self.rate = rospy.Rate(1/self.dt)

        # Class variables
        self.idx = 0  # current index in the trajectory
        self.N_traj = int(self.t_plan / self.dt)  # Length of trajectory
        self.X_nom_curr = None
        self.U_nom_curr = None
        self.X_nom_next = None
        self.U_nom_next = None

        self.x_hat = np.zeros((4,1))
        self.P = np.diag([0.01, 0.01, 0.001, 0.0])

        self.z = None
        self.v_des = 0
        self.t_start = 0

        self.new_traj_flag = False

        # Publishers
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.state_est_pub = rospy.Publisher('controller/state_est', State, queue_size=1)
        self.debug_pub = rospy.Publisher('debug/x_err', Float64, queue_size=1)

        # Subscribers
        traj_sub = rospy.Subscriber('planner/traj', NominalTrajectory, self.traj_callback)
        measurement_sub = rospy.Subscriber('sensing/mocap', State, self.measurement_callback)


    def traj_callback(self, data):
        """Trajectory subscriber callback.

        Save nominal states and controls from received trajectory.

        """
        self.new_traj_flag = True
        # If no current trajectory yet, set it
        if self.X_nom is None:
            self.X_nom_curr = data.states 
            self.U_nom_curr = data.controls 
        # Otherwise, set next trajectory
        else:
            self.X_nom_next = data.states 
            self.U_nom_next = data.controls
        rospy.loginfo("Received trajectory of length %d", len(data.states))


    def measurement_callback(self, data):
        """Measurement subscriber callback.

        Save received measurement.

        """
        # For full state measurement (i.e. mocap)
        self.z = np.array([[data.x],[data.y],[data.theta],[data.v]])


    def track(self):
        """Track next point in the current trajectory, and run the EKF for estimation.

        TODO

        """
        print("idx ", self.idx, " ----------------------------------------")

        x_nom_msg = self.X_nom[self.idx]
        x_nom = np.array([[x_nom_msg.x],[x_nom_msg.y],[x_nom_msg.theta],[x_nom_msg.v]])
        u_nom_msg = self.U_nom[self.idx]
        u_nom = np.array([[u_nom_msg.omega],[u_nom_msg.a]])

        if self.idx == 0:
            self.x_hat = x_nom

        A,B,C,K = generate_robot_matrices(x_nom, u_nom, self.Q_lqr, self.R_lqr, self.dt)
        # K = np.array([[0, 0, 1, 0],
        #               [0, 0, 0, 1]])

        # ======== EKF Update ========
        self.x_hat, self.P = EKF_correction_step(self.x_hat, self.P, self.z, C, self.R_ekf)
        self.state_est_pub.publish(wrap_states(self.x_hat)[0])

        # ======== Apply feedback control law ========
        u = compute_control(x_nom, u_nom, self.z, K)

        # Create motor command msg
        motor_cmd = Twist()
        
        # Closed-loop
        self.v_des += self.dt * u[1][0]  # integrate acceleration
        motor_cmd.linear.x = v_to_PWM(self.v_des)
        motor_cmd.angular.z = omega_to_PWM(u[0][0])

        print(" - u_a: ", round(u[1][0],2), " u_w: ", round(u[0][0],2))
        print(" - lin PWM: ", round(motor_cmd.linear.x,2), ", ang PWM: ", round(motor_cmd.angular.z,2))

        self.cmd_pub.publish(motor_cmd)

        self.idx += 1

        # ======== Check for end of trajectory ========
        if self.idx >= self.N_traj:
            # Finished tracking current trajectory 
            rospy.loginfo("Finished tracking trajectory")

            # If we have a next trajectory, switch to tracking that. Otherwise, continue the trajectory (braking maneuver)
            if self.X_nom_next is not None:
                self.X_nom_curr = self.X_nom_next
                self.U_nom_curr = self.U_nom_next
                self.X_nom_next = None
                self.X_nom_next = None
                self.idx = 0
            else:
                rospy.loginfo("Executing braking maneuver")

        # ======== Check for end of braking maneuver ========
        if self.idx >= len(self.U_nom_curr):
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
        self.x_hat, self.P = EKF_prediction_step(self.x_hat, u, self.P, A, self.Q_ekf, self.dt)

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
        rospy.loginfo("Running Trajectory Tracker")
        while not rospy.is_shutdown():
            
            if self.X_nom is not None:
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
