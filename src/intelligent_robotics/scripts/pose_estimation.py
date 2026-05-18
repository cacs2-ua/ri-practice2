#!/usr/bin/env python3

# Importar las librerías necesarias
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import ackermann_msgs.msg

# TODO Declare the mediapipe pose detector to be used

# Control message publisher
ackermann_command_publisher = None

#Operator image processing
def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convert ROS image to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # TODO Processing the image with MediaPipe

    # TODO Recognise the gesture by means of some classification from the landmarks.
    
    # TODO Draw landsmarks on the image

    # Display image with detected landmarks/gestures
    cv2.imshow("Hand pose Estimation", cv_image)
    cv2.waitKey(1)

    # TODO Interpret the obtained gesture and send the ackermann control command.

def main():
    rospy.init_node('pose_estimation', anonymous=True)
    rospy.Subscriber("/operator/image", Image, image_callback)

    ## Publisher definition
    ackermann_command_publisher = rospy.Publisher(
            "/blue/preorder_ackermann_cmd",
            ackermann_msgs.msg.AckermannDrive,
            queue_size=10,
        )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
