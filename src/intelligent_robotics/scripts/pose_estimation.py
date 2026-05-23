#!/usr/bin/env python3

# ---------------------------------------------------------------------------
# Block 1 - Import required libraries.
# This ROS node receives the operator image from /operator/image, detects hand
# landmarks using MediaPipe, recognises simple gestures, and publishes the
# corresponding Ackermann command in /blue/preorder_ackermann_cmd.
# ---------------------------------------------------------------------------
import math
import threading

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import mediapipe as mp
import ackermann_msgs.msg
import numpy as np


# ---------------------------------------------------------------------------
# Block 2 - MediaPipe hand detector declaration.
# This completes the original TODO and uses MediaPipe Hands, which provides the
# 21 hand landmarks required by the practice specification.
# ---------------------------------------------------------------------------
# TODO Declare the mediapipe pose detector to be used
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

hand_landmark_detector = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    model_complexity=1,
    min_detection_confidence=0.60,
    min_tracking_confidence=0.50
)


# ---------------------------------------------------------------------------
# Block 3 - Global ROS/OpenCV objects.
# The publisher is initialised in main(). The latest frame is stored by the
# callback and displayed by the main loop to avoid OpenCV window freezes.
# ---------------------------------------------------------------------------
# Control message publisher
ackermann_command_publisher = None

image_bridge = CvBridge()
latest_processed_frame = None
latest_frame_lock = threading.Lock()

gesture_window_name = "Hand Pose Estimation"


# ---------------------------------------------------------------------------
# Block 4 - Gesture-control parameters.
# These values are conservative so the robot does not move too aggressively.
# They can be changed using ROS private parameters if needed.
# ---------------------------------------------------------------------------
DEFAULT_FORWARD_SPEED = 1.30
MINIMUM_FORWARD_SPEED = 0.65
MAXIMUM_FORWARD_SPEED = 2.60
TURN_STEERING_ANGLE = 0.90

INDEX_DIRECTION_THRESHOLD = 0.06
EXTENDED_FINGER_ANGLE_THRESHOLD_DEGREES = 150.0


# ---------------------------------------------------------------------------
# Block 5 - Utility function to clamp numeric values.
# This keeps speed and steering values within safe limits.
# ---------------------------------------------------------------------------
def clamp(value, minimum_value, maximum_value):
    return max(min(value, maximum_value), minimum_value)


# ---------------------------------------------------------------------------
# Block 6 - Utility function to compute a 2D joint angle.
# The practice asks to process hand coordinates to obtain angular measurements.
# This function calculates the angle at the middle landmark using three points.
# ---------------------------------------------------------------------------
def calculate_joint_angle_degrees(first_point, middle_point, last_point):
    first_vector = np.array(first_point) - np.array(middle_point)
    last_vector = np.array(last_point) - np.array(middle_point)

    first_norm = np.linalg.norm(first_vector)
    last_norm = np.linalg.norm(last_vector)

    if first_norm < 1e-6 or last_norm < 1e-6:
        return 0.0

    cosine_angle = np.dot(first_vector, last_vector) / (first_norm * last_norm)
    cosine_angle = clamp(cosine_angle, -1.0, 1.0)

    return math.degrees(math.acos(cosine_angle))


# ---------------------------------------------------------------------------
# Block 7 - Helper function to read landmark coordinates.
# MediaPipe provides normalised image coordinates, which are sufficient for
# gesture recognition based on relative hand geometry.
# ---------------------------------------------------------------------------
def get_landmark_xy(hand_landmarks, landmark_index):
    landmark = hand_landmarks.landmark[landmark_index]
    return np.array([landmark.x, landmark.y])


# ---------------------------------------------------------------------------
# Block 8 - Finger extension analysis using joint angles.
# A finger is considered extended when its main joint angle is sufficiently
# open. This directly addresses the specification requirement of using angular
# measurements between hand joints.
# ---------------------------------------------------------------------------
def get_finger_extension_states(hand_landmarks):
    thumb_angle = calculate_joint_angle_degrees(
        get_landmark_xy(hand_landmarks, 2),
        get_landmark_xy(hand_landmarks, 3),
        get_landmark_xy(hand_landmarks, 4)
    )

    index_angle = calculate_joint_angle_degrees(
        get_landmark_xy(hand_landmarks, 5),
        get_landmark_xy(hand_landmarks, 6),
        get_landmark_xy(hand_landmarks, 8)
    )

    middle_angle = calculate_joint_angle_degrees(
        get_landmark_xy(hand_landmarks, 9),
        get_landmark_xy(hand_landmarks, 10),
        get_landmark_xy(hand_landmarks, 12)
    )

    ring_angle = calculate_joint_angle_degrees(
        get_landmark_xy(hand_landmarks, 13),
        get_landmark_xy(hand_landmarks, 14),
        get_landmark_xy(hand_landmarks, 16)
    )

    pinky_angle = calculate_joint_angle_degrees(
        get_landmark_xy(hand_landmarks, 17),
        get_landmark_xy(hand_landmarks, 18),
        get_landmark_xy(hand_landmarks, 20)
    )

    finger_states = {
        "thumb": thumb_angle > EXTENDED_FINGER_ANGLE_THRESHOLD_DEGREES,
        "index": index_angle > EXTENDED_FINGER_ANGLE_THRESHOLD_DEGREES,
        "middle": middle_angle > EXTENDED_FINGER_ANGLE_THRESHOLD_DEGREES,
        "ring": ring_angle > EXTENDED_FINGER_ANGLE_THRESHOLD_DEGREES,
        "pinky": pinky_angle > EXTENDED_FINGER_ANGLE_THRESHOLD_DEGREES,
    }

    finger_angles = {
        "thumb": thumb_angle,
        "index": index_angle,
        "middle": middle_angle,
        "ring": ring_angle,
        "pinky": pinky_angle,
    }

    return finger_states, finger_angles


# ---------------------------------------------------------------------------
# Block 9 - Select the main command hand.
# If two hands are detected, the largest hand in the image is used for direction
# commands and the other hand can be used for velocity modulation.
# ---------------------------------------------------------------------------
def calculate_hand_bounding_area(hand_landmarks):
    x_values = [landmark.x for landmark in hand_landmarks.landmark]
    y_values = [landmark.y for landmark in hand_landmarks.landmark]

    width = max(x_values) - min(x_values)
    height = max(y_values) - min(y_values)

    return width * height


def select_command_hand_index(multi_hand_landmarks):
    if not multi_hand_landmarks:
        return None

    hand_areas = [
        calculate_hand_bounding_area(hand_landmarks)
        for hand_landmarks in multi_hand_landmarks
    ]

    return int(np.argmax(hand_areas))


# ---------------------------------------------------------------------------
# Block 10 - Recognise the direction gesture from the command hand.
# The command hand supports:
#   - Open palm: move forward.
#   - Index finger pointing up: move forward.
#   - Index finger pointing left: turn left.
#   - Index finger pointing right: turn right.
#   - Closed/no recognised gesture: stop.
# ---------------------------------------------------------------------------
def recognise_direction_gesture(hand_landmarks):
    finger_states, finger_angles = get_finger_extension_states(hand_landmarks)

    extended_long_fingers = [
        finger_states["index"],
        finger_states["middle"],
        finger_states["ring"],
        finger_states["pinky"]
    ]

    number_of_extended_long_fingers = sum(extended_long_fingers)

    index_tip = get_landmark_xy(hand_landmarks, 8)
    index_mcp = get_landmark_xy(hand_landmarks, 5)
    index_direction_vector = index_tip - index_mcp

    horizontal_direction = index_direction_vector[0]
    vertical_direction = index_direction_vector[1]

    # Open palm: simple and robust forward command.
    if number_of_extended_long_fingers >= 4:
        return "forward", finger_angles

    # One-finger command mode: the index direction determines the command.
    if finger_states["index"] and number_of_extended_long_fingers <= 2:
        if abs(horizontal_direction) > abs(vertical_direction):
            if horizontal_direction < -INDEX_DIRECTION_THRESHOLD:
                return "left", finger_angles

            if horizontal_direction > INDEX_DIRECTION_THRESHOLD:
                return "right", finger_angles

        if vertical_direction < -INDEX_DIRECTION_THRESHOLD:
            return "forward", finger_angles

    return "stop", finger_angles


# ---------------------------------------------------------------------------
# Block 11 - Estimate velocity using the optional second hand.
# The distance between thumb tip and index tip of the second hand is used as a
# simple velocity modulus. If only one hand is detected, a default speed is used.
# ---------------------------------------------------------------------------
def estimate_velocity_from_second_hand(multi_hand_landmarks, command_hand_index):
    if not multi_hand_landmarks or len(multi_hand_landmarks) < 2:
        return DEFAULT_FORWARD_SPEED

    velocity_hand_index = 1 - command_hand_index
    velocity_hand_landmarks = multi_hand_landmarks[velocity_hand_index]

    thumb_tip = get_landmark_xy(velocity_hand_landmarks, 4)
    index_tip = get_landmark_xy(velocity_hand_landmarks, 8)
    wrist = get_landmark_xy(velocity_hand_landmarks, 0)
    middle_mcp = get_landmark_xy(velocity_hand_landmarks, 9)

    pinch_distance = np.linalg.norm(thumb_tip - index_tip)
    hand_reference_size = np.linalg.norm(wrist - middle_mcp)

    if hand_reference_size < 1e-6:
        return DEFAULT_FORWARD_SPEED

    normalised_pinch_distance = clamp(pinch_distance / hand_reference_size, 0.30, 1.40)
    velocity_ratio = (normalised_pinch_distance - 0.30) / (1.40 - 0.30)

    return MINIMUM_FORWARD_SPEED + velocity_ratio * (MAXIMUM_FORWARD_SPEED - MINIMUM_FORWARD_SPEED)


# ---------------------------------------------------------------------------
# Block 12 - Convert recognised gesture into Ackermann control command.
# The output message is published in /blue/preorder_ackermann_cmd. This is the
# command before the safety layer required in Part 6.
# ---------------------------------------------------------------------------
def build_ackermann_command_from_gesture(gesture_name, selected_speed, invert_steering=False):
    ackermann_command = ackermann_msgs.msg.AckermannDrive()

    if gesture_name == "forward":
        ackermann_command.speed = selected_speed
        ackermann_command.steering_angle = 0.0

    elif gesture_name == "left":
        ackermann_command.speed = selected_speed
        ackermann_command.steering_angle = TURN_STEERING_ANGLE

    elif gesture_name == "right":
        ackermann_command.speed = selected_speed
        ackermann_command.steering_angle = -TURN_STEERING_ANGLE

    else:
        ackermann_command.speed = 0.0
        ackermann_command.steering_angle = 0.0

    if invert_steering:
        ackermann_command.steering_angle *= -1.0

    return ackermann_command


# ---------------------------------------------------------------------------
# Block 13 - Draw information on the operator image.
# This is useful for experimentation and for the video demonstration required
# by the practice documentation.
# ---------------------------------------------------------------------------
def draw_gesture_information(image, gesture_name, selected_speed, steering_angle, finger_angles):
    cv2.putText(
        image,
        "Gesture: {}".format(gesture_name.upper()),
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 0),
        2
    )

    cv2.putText(
        image,
        "Speed: {:.2f} m/s | Steering: {:.2f} rad".format(selected_speed, steering_angle),
        (20, 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.70,
        (0, 255, 0),
        2
    )

    y_position = 120

    for finger_name, finger_angle in finger_angles.items():
        cv2.putText(
            image,
            "{} angle: {:.1f}".format(finger_name, finger_angle),
            (20, y_position),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1
        )
        y_position += 25


# ---------------------------------------------------------------------------
# Block 14 - Operator image processing callback.
# This keeps the initial function structure but completes all TODOs:
#   - MediaPipe processing.
#   - Gesture classification.
#   - Landmark drawing.
#   - Ackermann command publication.
# ---------------------------------------------------------------------------
#Operator image processing
def image_callback(msg):
    global ackermann_command_publisher, latest_processed_frame

    try:
        # Convert ROS image to OpenCV image
        cv_image = image_bridge.imgmsg_to_cv2(msg, "bgr8")

    except CvBridgeError as error:
        rospy.logerr("Could not convert operator image: %s", error)
        return

    invert_steering = rospy.get_param("~invert_steering", False)

    # TODO Processing the image with MediaPipe
    rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    rgb_image.flags.writeable = False
    detection_results = hand_landmark_detector.process(rgb_image)
    rgb_image.flags.writeable = True

    recognised_gesture = "stop"
    selected_speed = 0.0
    steering_angle = 0.0
    finger_angles = {}

    # TODO Recognise the gesture by means of some classification from the landmarks.
    if detection_results.multi_hand_landmarks:
        command_hand_index = select_command_hand_index(detection_results.multi_hand_landmarks)
        command_hand_landmarks = detection_results.multi_hand_landmarks[command_hand_index]

        recognised_gesture, finger_angles = recognise_direction_gesture(command_hand_landmarks)
        selected_speed = estimate_velocity_from_second_hand(
            detection_results.multi_hand_landmarks,
            command_hand_index
        )

        # TODO Interpret the obtained gesture and send the ackermann control command.
        ackermann_command = build_ackermann_command_from_gesture(
            recognised_gesture,
            selected_speed,
            invert_steering=invert_steering
        )

        steering_angle = ackermann_command.steering_angle

    else:
        # No hand detected means no movement command.
        ackermann_command = build_ackermann_command_from_gesture(
            "stop",
            0.0,
            invert_steering=invert_steering
        )

    if ackermann_command_publisher is not None:
        ackermann_command_publisher.publish(ackermann_command)

    # TODO Draw landsmarks on the image
    if detection_results.multi_hand_landmarks:
        for hand_landmarks in detection_results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(
                cv_image,
                hand_landmarks,
                mp_hands.HAND_CONNECTIONS,
                mp_drawing_styles.get_default_hand_landmarks_style(),
                mp_drawing_styles.get_default_hand_connections_style()
            )

    draw_gesture_information(
        cv_image,
        recognised_gesture,
        ackermann_command.speed,
        steering_angle,
        finger_angles
    )

    # Display image with detected landmarks/gestures
    # The latest processed frame is stored and displayed in the main loop. This
    # is more stable than calling cv2.imshow directly inside the callback.
    with latest_frame_lock:
        latest_processed_frame = cv_image.copy()


# ---------------------------------------------------------------------------
# Block 15 - Main OpenCV display loop.
# This loop keeps the gesture detection window responsive inside Docker.
# ---------------------------------------------------------------------------
def display_processed_operator_image():
    global latest_processed_frame

    cv2.namedWindow(gesture_window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(gesture_window_name, 960, 720)
    cv2.startWindowThread()

    display_rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        frame_to_show = None

        with latest_frame_lock:
            if latest_processed_frame is not None:
                frame_to_show = latest_processed_frame.copy()

        if frame_to_show is not None:
            cv2.imshow(gesture_window_name, frame_to_show)

        key = cv2.waitKey(10) & 0xFF

        if key == 27 or key == ord("q"):
            rospy.loginfo("Closing hand pose estimation window.")
            rospy.signal_shutdown("User closed hand pose estimation window.")
            break

        display_rate.sleep()

    cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# Block 16 - Main ROS node initialisation.
# The node subscribes to /operator/image and publishes the pre-safety Ackermann
# command in /blue/preorder_ackermann_cmd.
# ---------------------------------------------------------------------------
def main():
    global ackermann_command_publisher

    rospy.init_node('pose_estimation', anonymous=True)

    operator_image_topic = rospy.get_param("~operator_image_topic", "/operator/image")
    preorder_ackermann_topic = rospy.get_param(
        "~preorder_ackermann_topic",
        "/blue/preorder_ackermann_cmd"
    )

    rospy.loginfo("Starting MediaPipe hand pose estimation node.")
    rospy.loginfo("Subscribing to operator image topic: %s", operator_image_topic)
    rospy.loginfo("Publishing pre-safety Ackermann commands to: %s", preorder_ackermann_topic)

    rospy.Subscriber(
        operator_image_topic,
        Image,
        image_callback,
        queue_size=1,
        buff_size=2**24
    )

    ## Publisher definition
    ackermann_command_publisher = rospy.Publisher(
            preorder_ackermann_topic,
            ackermann_msgs.msg.AckermannDrive,
            queue_size=10,
        )

    try:
        display_processed_operator_image()

    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# Block 17 - Python entry point.
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    main()