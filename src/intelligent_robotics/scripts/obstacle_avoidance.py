#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from ackermann_msgs.msg import AckermannDrive
import numpy as np

class ObstacleAvoidance:
    def __init__(self):
        # Initialise the ROS node
        rospy.init_node('obstacle_avoidance')

        # Subscriber for the point cloud of obstacles captured by lidar
        rospy.Subscriber("/obstacles", PointCloud2, self.obstacle_callback)

        # Subscriber for Ackermann control commands
        rospy.Subscriber("/blue/preorder_ackermann_cmd", AckermannDrive, self.ackermann_callback)

        # TODO Publisher for modified Ackermann commands

        # Store the last Ackermann message received
        self.last_ackermann_cmd = AckermannDrive()

    def obstacle_callback(self, msg):
        # TODO Process the point cloud with the obstacles taking into account the last received movement message to avoid collisions.

        #Send ackermann's message 
        cmd = self.last_ackermann_cmd

        # TODO Modify ackermann's message if necessary
        self.cmd_pub.publish(cmd)
        
        return

    def ackermann_callback(self, msg):
        # Stores the last command received
        self.last_ackermann_cmd = msg

    def modify_ackermann_command(self):
        # Modify the Ackermann command to avoid obstacles (can be modified if necessary).
        cmd = self.last_ackermann_cmd
        cmd.drive.speed = 0.0  # Reduce speed
        cmd.drive.steering_angle = 0.0 # Change the steering angle
        

if __name__ == '__main__':
    oa = ObstacleAvoidance()
    rospy.spin()
