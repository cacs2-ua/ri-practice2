#!/usr/bin/env python3

# This node implements the safety layer required in Part 6. It receives the
# gesture command from /blue/preorder_ackermann_cmd, checks the obstacle point
# cloud from /obstacles, and publishes the final safe command in
# /blue/ackermann_cmd.
import copy
import math

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from ackermann_msgs.msg import AckermannDrive
import numpy as np


# It limits numerical values so the safety layer never publishes unsafe
# steering or speed values.
def clamp(value, minimum_value, maximum_value):
    return max(min(value, maximum_value), minimum_value)


class ObstacleAvoidance:
    # This keeps the original class structure, subscribers and DONE locations,
    # while adding the required publisher and safety parameters.
    def __init__(self):
        # Initialise the ROS node
        rospy.init_node('obstacle_avoidance')

        # Safety configuration. These parameters can be tuned without changing
        # the file by using rosrun/roslaunch private parameters.
        self.vehicle_length = rospy.get_param("~vehicle_length", 1.05)
        self.vehicle_half_width = rospy.get_param("~vehicle_half_width", 0.45)

        self.minimum_detection_distance = rospy.get_param("~minimum_detection_distance", 0.35)
        self.maximum_detection_distance = rospy.get_param("~maximum_detection_distance", 2.20)
        self.collision_lateral_margin = rospy.get_param("~collision_lateral_margin", 0.35)

        self.maximum_allowed_speed = rospy.get_param("~maximum_allowed_speed", 0.90)
        self.maximum_allowed_reverse_speed = rospy.get_param("~maximum_allowed_reverse_speed", 0.60)
        self.maximum_allowed_steering_angle = rospy.get_param("~maximum_allowed_steering_angle", 0.42)

        # Reverse obstacle detection can be configured independently from forward
        # obstacle detection. Usually it is safer to use a shorter distance backwards.
        self.maximum_reverse_detection_distance = rospy.get_param(
            "~maximum_reverse_detection_distance",
            min(self.maximum_detection_distance, 2.00)
        )

        self.obstacle_timeout = rospy.get_param("~obstacle_timeout", 0.80)
        self.command_timeout = rospy.get_param("~command_timeout", 0.80)
        self.publish_rate_hz = rospy.get_param("~publish_rate", 20.0)

        self.obstacle_topic = rospy.get_param("~obstacle_topic", "/obstacles")
        self.preorder_ackermann_topic = rospy.get_param(
            "~preorder_ackermann_topic",
            "/blue/preorder_ackermann_cmd"
        )
        self.safe_ackermann_topic = rospy.get_param(
            "~safe_ackermann_topic",
            "/blue/ackermann_cmd"
        )

        # Runtime state.
        self.last_ackermann_cmd = AckermannDrive()
        self.last_command_time = rospy.Time(0)

        self.collision_risk_detected = False
        self.closest_obstacle_distance = float("inf")
        self.last_obstacle_time = rospy.Time(0)

        # Subscriber for the point cloud of obstacles captured by lidar
        rospy.Subscriber(
            self.obstacle_topic,
            PointCloud2,
            self.obstacle_callback,
            queue_size=1
        )

        # Subscriber for Ackermann control commands
        rospy.Subscriber(
            self.preorder_ackermann_topic,
            AckermannDrive,
            self.ackermann_callback,
            queue_size=10
        )

        # DONE Publisher for modified Ackermann commands
        self.cmd_pub = rospy.Publisher(
            self.safe_ackermann_topic,
            AckermannDrive,
            queue_size=10
        )

        # Periodic publisher. This makes the safety layer robust even when
        # obstacle callbacks and gesture callbacks arrive at different rates.
        self.publish_timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate_hz),
            self.publish_safe_command
        )

        rospy.loginfo("Obstacle avoidance safety layer started.")
        rospy.loginfo("Subscribing to obstacles: %s", self.obstacle_topic)
        rospy.loginfo("Subscribing to pre-safety commands: %s", self.preorder_ackermann_topic)
        rospy.loginfo("Publishing final safe commands: %s", self.safe_ackermann_topic)

    # Given an obstacle point in the robot/LiDAR frame and the current steering
    # command, this function estimates whether the obstacle lies inside the
    # path corridor that the robot is about to follow.
    def is_obstacle_in_commanded_path(self, obstacle_x, obstacle_y, speed, steering_angle):
        # If the robot is not moving, no movement path has to be blocked.
        if abs(speed) < 1e-4:
            return False

        # Movement direction:
        #   speed > 0 -> check obstacles in front of the robot
        #   speed < 0 -> check obstacles behind the robot
        if speed > 0.0:
            movement_direction = 1.0
            maximum_distance = self.maximum_detection_distance
        else:
            movement_direction = -1.0
            maximum_distance = self.maximum_reverse_detection_distance

        # In the Velodyne frame, x > 0 is normally in front of the robot and
        # x < 0 is behind the robot. Multiplying by movement_direction converts
        # the relevant movement direction into a positive longitudinal distance.
        longitudinal_distance = obstacle_x * movement_direction
        lateral_distance = obstacle_y

        # Ignore points that are not in the movement direction.
        if longitudinal_distance < self.minimum_detection_distance:
            return False

        if longitudinal_distance > maximum_distance:
            return False

        # Bicycle-model-inspired local path approximation:
        #   curvature = tan(delta) / L
        #   y_path ≈ 0.5 * curvature * s^2
        #
        # This is used both forward and backward. For reverse commands, s is the
        # positive distance behind the robot after the direction conversion above.
        steering_angle = clamp(
            steering_angle,
            -self.maximum_allowed_steering_angle,
            self.maximum_allowed_steering_angle
        )

        if abs(steering_angle) < 1e-4:
            predicted_path_y = 0.0
        else:
            curvature = math.tan(steering_angle) / max(self.vehicle_length, 1e-3)
            predicted_path_y = 0.5 * curvature * (longitudinal_distance ** 2)

        allowed_lateral_distance = self.vehicle_half_width + self.collision_lateral_margin

        return abs(lateral_distance - predicted_path_y) <= allowed_lateral_distance
    # This completes the DONE that asks to process /obstacles considering the
    # last movement command. If at least one point lies in the commanded path,
    # the safety layer marks the command as unsafe.
    def obstacle_callback(self, msg):
        # DONE Process the point cloud with the obstacles taking into account the last received movement message to avoid collisions.
        current_speed = self.last_ackermann_cmd.speed
        current_steering_angle = self.last_ackermann_cmd.steering_angle

        collision_risk_detected = False
        closest_obstacle_distance = float("inf")

        obstacle_points = pc2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        )

        for obstacle_point in obstacle_points:
            obstacle_x, obstacle_y, _ = obstacle_point
            obstacle_distance = np.hypot(obstacle_x, obstacle_y)

            if self.is_obstacle_in_commanded_path(
                obstacle_x,
                obstacle_y,
                current_speed,
                current_steering_angle
            ):
                collision_risk_detected = True
                closest_obstacle_distance = min(
                    closest_obstacle_distance,
                    obstacle_distance
                )

        self.collision_risk_detected = collision_risk_detected
        self.closest_obstacle_distance = closest_obstacle_distance
        self.last_obstacle_time = rospy.Time.now()

        if self.collision_risk_detected:
            rospy.logwarn_throttle(
                0.5,
                "Collision risk detected. Closest obstacle distance: %.2f m",
                self.closest_obstacle_distance
            )

        return

    # The gesture recognition node publishes here. This callback stores the
    # latest received command and applies basic speed/steering limits.
    def ackermann_callback(self, msg):
        # Stores the last command received
        safe_input_command = AckermannDrive()
        safe_input_command.speed = clamp(
            msg.speed,
            -self.maximum_allowed_reverse_speed,
            self.maximum_allowed_speed
        )
        safe_input_command.steering_angle = clamp(
            msg.steering_angle,
            -self.maximum_allowed_steering_angle,
            self.maximum_allowed_steering_angle
        )

        safe_input_command.acceleration = msg.acceleration
        safe_input_command.jerk = msg.jerk
        safe_input_command.steering_angle_velocity = msg.steering_angle_velocity

        self.last_ackermann_cmd = safe_input_command
        self.last_command_time = rospy.Time.now()

    # This completes the DONE that asks to modify the Ackermann command if
    # necessary. If an obstacle is in the commanded path, speed and steering are
    # set to zero to stop the robot.
    def modify_ackermann_command(self):
        # Modify the Ackermann command to avoid obstacles (can be modified if necessary).
        cmd = copy.deepcopy(self.last_ackermann_cmd)

        cmd.speed = 0.0  # Reduce speed
        cmd.steering_angle = 0.0 # Change the steering angle
        cmd.acceleration = 0.0
        cmd.jerk = 0.0
        cmd.steering_angle_velocity = 0.0

        return cmd

    # Block 8 - Build the final safe command.
    # If obstacles are recent and the path is blocked, publish a stop command.
    # If the path is free, forward the original gesture command.
    def build_safe_ackermann_command(self):
        current_time = rospy.Time.now()

        time_since_last_command = (current_time - self.last_command_time).to_sec()
        time_since_last_obstacle_msg = (current_time - self.last_obstacle_time).to_sec()

        # If no recent command exists, stop the robot.
        if time_since_last_command > self.command_timeout:
            return AckermannDrive()

        # If obstacle information is stale, fail safe and stop instead of
        # forwarding a possibly dangerous command.
        if time_since_last_obstacle_msg > self.obstacle_timeout:
            rospy.logwarn_throttle(
                1.0,
                "Obstacle information is stale. Stopping robot for safety."
            )
            return self.modify_ackermann_command()

        # DONE Modify ackermann's message if necessary
        if self.collision_risk_detected:
            return self.modify_ackermann_command()

        return copy.deepcopy(self.last_ackermann_cmd)

    # This publishes to /blue/ackermann_cmd, which is the topic consumed by the
    # BLUE robot low-level control bridge.
    def publish_safe_command(self, event):
        #Send ackermann's message
        cmd = self.build_safe_ackermann_command()
        self.cmd_pub.publish(cmd)


if __name__ == '__main__':
    oa = ObstacleAvoidance()
    rospy.spin()