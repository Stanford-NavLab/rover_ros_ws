#!/usr/bin/env python

import rospy
import numpy as np
import csv
import os

from geometry_msgs.msg import Twist

from trajectory_msgs.msg import JointTrajectory
from controller.controller_utils import v_to_PWM  # TODO: mec_v_to_PWM
import params.rtd_params as params
import rtd.utils as utils


class linear_tracker():
    """Linear tracker

    Tracks nominal trajectories for linear 2D double-integrator robot. 
    Controls are x and y accelerations.
    Used for RTD Linear Planner with Rover in mecanum drive

    Attributes
    ----------
    traj : Trajectory
        Current trajectory 

    """
    def __init__(self):
        # Initialize node 
        rospy.init_node('linear_tracker', anonymous=True)
        self.rate = rospy.Rate(1/params.DT)

        # Class variables
        self.idx = 0  # current index in the trajectory
        self.traj = None

        self.v_des = np.zeros(2)
        self.t_start = 0

        # Publishers
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Subscribers
        traj_sub = rospy.Subscriber('planner/traj', JointTrajectory, self.traj_callback)
        # measurement_sub = rospy.Subscriber('sensing/mocap_noisy', State, self.measurement_callback)
        # gt_sub = rospy.Subscriber('sensing/mocap', State, self.gt_callback)

        # Logging
        path = '/home/navlab-nuc/multirobot-planning/data/ros_sim_runs'
        filename = 'trajectory_'+str(rospy.get_time())+'.csv'
        self.logger = csv.writer(open(os.path.join(path, filename), 'w'))
        self.logger.writerow(['t', 'x_nom', 'y_nom'])


    def traj_callback(self, data):
        """Trajectory subscriber callback.

        Save received trajectory.

        """
        # Store new trajectory
        time = np.linspace(0, params.TRAJ_TIME_LEN, params.TRAJ_IDX_LEN)  # TODO: update time to not always start at 0
        self.traj = utils.unwrap_2D_traj_msg(data, time)
        # Reset index
        self.idx = 0
        rospy.loginfo("Received trajectory")


    def track(self):
        """Track next point in the current trajectory.

        TODO

        """
        print("idx ", self.idx, " ----------------------------------------")

        x_nom = self.traj.positions[:,self.idx]
        u_nom = self.traj.accelerations[:,self.idx]

        # Create motor command msg
        motor_cmd = Twist()
        
        # Open-loop
        self.v_des += params.DT * u_nom  # integrate acceleration
        motor_cmd.linear.x = 0.2 * self.v_des[0]  # TODO: mec_v_to_PWM
        motor_cmd.linear.y = 0.2 * self.v_des[1]

        print(" - x_nom: ", np.round(x_nom,2))
        print(" - v_des: ", np.round(self.v_des,2), " u_x: ", round(u_nom[0],2), " u_y: ", round(u_nom[1],2))
        print(" - x PWM: ", round(motor_cmd.linear.x,2), ", y PWM: ", round(motor_cmd.linear.y,2))

        # print(" - v_des: ", np.round(self.v_des,2), " u_x: ", np.round(u_nom[0],2), " u_y: ", np.round(u_nom[1],2))
        # print(" - x PWM: ", round(motor_cmd.linear.x,2), ", y PWM: ", round(motor_cmd.linear.y,2))

        self.cmd_pub.publish(motor_cmd)

        self.idx += 1

        # Log data
        self.logger.writerow([rospy.get_time(), x_nom[0], x_nom[1]])

        # ======== Check for end of trajectory ========
        if self.idx >= params.TRAJ_IDX_LEN:
            rospy.loginfo("Reached end of current trajectory - braking")
            # Reset class variables
            self.traj = None
            self.v_des = 0
            self.idx = 0

            # Send multiple stop commands in case some don't go through
            for i in range(5):
                self.rate.sleep()
                self.stop_motors()

    
    def stop_motors(self):
        """Send stop command to all motors

        """
        rospy.loginfo("Stopping motors")
        motor_cmd = Twist()
        motor_cmd.linear.x = 0.0
        motor_cmd.linear.y = 0.0
        self.cmd_pub.publish(motor_cmd)


    def run(self):
        """Run node

        """
        rospy.loginfo("Running Linear Tracker")
        while not rospy.is_shutdown():
            
            if self.traj is not None:
                self.track()

            self.rate.sleep()
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    lt = linear_tracker()
    try:
        lt.run()
    except rospy.ROSInterruptException:
        pass
