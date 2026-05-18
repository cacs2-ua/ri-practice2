#!/usr/bin/env python3

# Import required libraries
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Callback function for when an image is received
def image_callback(msg):
    try:
        # Converting the ROS message to an OpenCV image
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        # TODO Display the image in an OpenCV window and wait 1 ms for OpenCV to process the GUI events.
        return

def main():
    rospy.init_node('image_viewer', anonymous=True)
    # TODO Subscribe to ROS topic that has the images
    
    # Preventing Python from shutting down until the node is stopped
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
