#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import sys
import os
import json

from std_msgs.msg import Bool

from planner.msg import State, Control, NominalTrajectory
import planner.planner_utils as plan_util
from controller.controller_utils import wrap_angle
import params.params as params


class TrajPublisher():
    """TrajPublisher

    Node which publishes a trajectory to be tracked from an input file.

    Attributes
    ----------
    TODO

    Methods
    -------
    TODO

    """
    def __init__(self):
        # Parameters (eventually make global)
        # self.max_acc_mag = 0.25
        # self.t_plan = 3.0  # time duration of each trajectory
        # self.t_next = 3.0  # time between sending trajectories
        # self.dt = 0.2
        self.HOME_X = -5.0
        self.HOME_Y = 0.0

        # Initialize node 
        rospy.init_node('traj_publisher', anonymous=True, disable_signals=True)
        self.rate = rospy.Rate(10)

        # Subscribers
        self.mocap_sub = rospy.Subscriber('sensing/mocap', State, self.gt_callback)
        self.done_sub = rospy.Subscriber('controller/done', Bool, self.done_callback)

        # Publishers
        self.traj_pub = rospy.Publisher('planner/traj', NominalTrajectory, queue_size=10)

        self.x_nom_end = params.X_0
        self.traj_msg = None
        self.z_gt = np.zeros((3,1))

        # Load trajectory file
        filename = 'run_69.json'
        with open(os.path.join(rospkg.RosPack().get_path('planner'), 'files', filename), 'r') as f:
            rundict = json.loads(json.load(f))

        # Create trajectory msg
        x_nom = np.array(rundict['trajectory'])
        u_nom = np.array(rundict['controls'])

        # Reverse trajectory
        print("REVERSING!!!!")
        x_nom = x_nom[:, ::-1]
        x_nom[3, :] = -x_nom[3, :]
        u_nom = -u_nom[:, ::-1]

        self.traj_msg = NominalTrajectory()
        self.traj_msg.states = plan_util.wrap_states(x_nom)
        self.traj_msg.controls = plan_util.wrap_controls(u_nom)

        x_nom_reverse = x_nom[:,::-1].copy()
        x_nom_reverse[-1,:] = -x_nom_reverse[-1,:]
        u_nom_reverse = -u_nom[:,::-1].copy()

        # x_nom = np.concatenate((x_nom, x_nom_reverse), axis=-1)
        # u_nom = np.concatenate((u_nom, u_nom_reverse), axis=-1)

        self.traj_msg_reverse = NominalTrajectory()
        self.traj_msg_reverse.states = plan_util.wrap_states(x_nom_reverse)
        self.traj_msg_reverse.controls = plan_util.wrap_controls(u_nom_reverse)


    def gt_callback(self, data):
        """Ground-truth subscriber callback.

        Save received ground-truth.

        """
        # Mocap data
        self.z_gt = np.array([[data.x],[data.y],[data.theta]])
    

    def done_callback(self, data):
        """Done subscriber callback.

        Send next trajectory.

        """
        #self.traj_pub.publish(self.traj_msg_reverse)


    def reset_traj(self):
        """Determine reset trajectory

        """
        # Determine heading from current position to home position
        w = 0.2  # turning rate [rad/s] 
        N = 25  # 5 seconds 
        x_nom = np.zeros((4, N))  # (4 x N) x,y,theta,v
        u_nom = np.zeros((2, N))  # (2 x N) w,a
        x_nom[:2,:] = self.z_gt[:2]
        
        theta_0 = np.arctan2(self.z_gt[1] - self.HOME_Y, self.z_gt[0] - self.HOME_X)
        theta_des = wrap_angle(np.pi - self.z_gt[2] + theta_0)
        thetas = wrap_angle(np.linspace(self.z_gt[2], theta_des, N))
        x_nom[2,:] = thetas[:,0]
        u_nom[0,:] = w

        print(x_nom)

        # Rotate to heading
        rotate_msg = NominalTrajectory()
        rotate_msg.states = plan_util.wrap_states(x_nom)
        rotate_msg.controls = plan_util.wrap_controls(u_nom)
        self.traj_pub.publish(rotate_msg)

        # Drive to home
        # Rotate back to home orientation



    def generate_straight_traj(self):
        N = 100  # 5 seconds 
        v_max = 0.5 # 1 m/s
        x_nom = np.zeros((4, N))
        u_nom = np.zeros((2, N)) 
        
        # x_nom[0,:] = np.linspace(self.HOME_X, 5, N)
        # x_nom[3,:25] = np.linspace(0, v_max, 25)
        # x_nom[3,25:90] = v_max
        # x_nom[3,90:] = np.linspace(v_max, 0.15, 10)
        
        # u_nom[1,:25] = np.linspace(0, 0.5, 25)
        # u_nom[1,75:] = np.linspace(-0.3, 0, 25)
        # u_nom[1,:] = np.array([0.0, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.2334375083446505, 0.0, 0.0, 0.0, 0.0, 0.0, -0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.25])[:100]
        # u_nom[1,:] = 0.5
        # u_nom[1,12:] = -0.5
        
        # REVERSE
        # x_nom = -x_nom
        # u_nom = -u_nom

        # TURN IN-PLACE
        x_nom[0,:] = self.HOME_X    # x
        x_nom[1,:] = 0.0    # y
        x_nom[2,:] = np.linspace(0, np.pi, N)    # theta
        x_nom[3,:] = 0.0    # v

        u_nom[0,:N//4] = np.linspace(0, 0.01, N//4)  # kw


        line_msg = NominalTrajectory()
        line_msg.states = plan_util.wrap_states(x_nom)
        line_msg.controls = plan_util.wrap_controls(u_nom)
        self.traj_pub.publish(line_msg)
        

    def publish_trajectory(self):
        """Publish trajectory message

        """
        self.traj_pub.publish(self.traj_msg)


    def run(self):
        """Run node

        """
        rospy.loginfo("Running Traj Publisher")
        
        published = False

        while not rospy.is_shutdown():
            connections = self.traj_pub.get_num_connections()
            rospy.loginfo("Connections: %d", connections)

            if connections > 0:
                if not published:
                    self.publish_trajectory()
                    # self.generate_straight_traj()
                    published = True
                #self.reset_traj()
                #rospy.loginfo("Published trajectory")
                #rospy.loginfo("Exiting...")
                #rospy.signal_shutdown("")

            self.rate.sleep()
            
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    tp = TrajPublisher()
    try:
        tp.run()
    except rospy.ROSInterruptException:
        pass
