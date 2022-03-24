#!/usr/bin/env python3

import rospy
import numpy as np
import time

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist, PoseStamped

from planner.msg import State, Control, NominalTrajectory
from controller.controller_utils import compute_control, v_to_PWM, omega_to_PWM
from planner.planner_utils import generate_robot_matrices

class traj_tracker():
    """Trajectory tracker

    Tracks nominal trajectories by applying linear control feedback using 
    state estimate and sending motor commands.

    """
    def __init__(self):
        # parameters (eventually make these global)
        self.dt = 0.2
        self.t_plan = 3.0
        self.Q_lqr = np.diag(np.array([5, 5, 10, 100]))
        self.R_lqr = np.diag(np.array([10, 1]))

        # initialize node 
        rospy.init_node('traj_tracker', anonymous=True)
        self.rate = rospy.Rate(1/self.dt)

        # class variables
        self.idx = 0 # current index in the trajectory
        self.X_nom = None
        self.U_nom = None
        self.x_hat = np.zeros((4,1))
        self.v_des = 0
        self.t_start = 0

        # publishers
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # subscribers
        traj_sub = rospy.Subscriber('planner/traj', NominalTrajectory, self.traj_callback)
        state_est_sub = rospy.Subscriber('controller/state_est', State, self.state_est_callback)


    def traj_callback(self, data):
        """Trajectory subscriber callback

        Save nominal states and controls from received trajectory.

        """
        self.X_nom = data.states 
        self.U_nom = data.controls 
        rospy.loginfo("Received trajectory of length %d", len(self.X_nom))
        self.t_start = time.time()


    def state_est_callback(self, data):
        """State estimator subscriber callback

        Save received state estimate.

        """
        self.x_hat = np.array([[data.x],[data.y],[data.theta],[data.v]])

    
    def track(self):
        """Track 

        Track the new point in the current trajectory.

        """
        # Apply feedback control law
        x_nom_msg = self.X_nom[self.idx]
        x_nom = np.array([[x_nom_msg.x],[x_nom_msg.y],[x_nom_msg.theta],[x_nom_msg.v]])
        u_nom_msg = self.U_nom[self.idx]
        u_nom = np.array([[u_nom_msg.omega],[u_nom_msg.a]])

        #_,_,_,K = generate_robot_matrices(x_nom, u_nom, self.Q_lqr, self.R_lqr, self.dt)
        K = np.array([[0.5, 0.5, 1, 0],
                      [0, 0, 0, 1]])
        u = compute_control(x_nom, u_nom, self.x_hat, K)

        print("idx ", self.idx, " - w: ", u[0][0], " a: ", u[1][0])

        # Create motor command msg
        motor_cmd = Twist()

        # open-loop
        # if u_nom_msg.a == -np.inf:  # check for braking maneuver
        #     self.v_des = 0
        # else:
        #self.v_des += self.dt * u_nom_msg.a  # integrate acceleration
        #motor_cmd.linear.x = v_to_PWM(self.v_des)
        #motor_cmd.angular.z = omega_to_PWM(u_nom_msg.omega)

        #print("desired linear velocity: ", self.v_des, " m/s")
        
        # closed-loop
        self.v_des += self.dt * u[1][0]  # integrate acceleration
        motor_cmd.linear.x = v_to_PWM(self.v_des)
        motor_cmd.angular.z = omega_to_PWM(u[0][0])

        print("idx ", self.idx, " - linear PWM: ", motor_cmd.linear.x, ", angular PWM: ", motor_cmd.angular.z)

        self.cmd_pub.publish(motor_cmd)

        #rospy.loginfo("Tracking trajectory, idx = %d", self.idx)

        self.idx += 1
        if self.idx >= len(self.U_nom):
            # Finished tracking current trajectory (means we need to execute braking maneuver)
            rospy.loginfo("Finished tracking trajectory")
            print("elapsed time: ", time.time() - self.t_start)

            # Reset class variables
            self.X_nom = None
            self.U_nom = None
            self.v_des = 0
            self.idx = 0

            # Sleep before sending stop to allow buffer time from last command
            #self.rate.sleep()
            for i in range(5):
                self.rate.sleep()
                self.stop_motors()
            

    
    def stop_motors(self):
        """Stop motors
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
