#!/usr/bin/env python3

# ---------------------------------------------------------------------------
# Block 1 - Import required libraries.
# This node subscribes to the simulated robot environment camera topic and
# displays the received image in an OpenCV window.
# ---------------------------------------------------------------------------
import threading

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


# ---------------------------------------------------------------------------
# Block 2 - Global variables shared between the ROS callback and the OpenCV
# display loop.
# The callback only stores the latest frame. The main loop displays it. This
# avoids freezing the OpenCV window inside the ROS callback.
# ---------------------------------------------------------------------------
image_bridge = CvBridge()
latest_camera_frame = None
latest_frame_lock = threading.Lock()

camera_window_name = "BLUE Robot Environment Camera"


# ---------------------------------------------------------------------------
# Block 3 - Callback function for when an image is received.
# The callback converts the ROS Image message into an OpenCV image and stores
# only the latest frame.
# ---------------------------------------------------------------------------
def image_callback(msg):
    global latest_camera_frame

    try:
        # Converting the ROS message to an OpenCV image.
        cv_image = image_bridge.imgmsg_to_cv2(msg, "bgr8")

    except CvBridgeError as error:
        rospy.logerr("Could not convert ROS image to OpenCV image: %s", error)
        return

    # Store the last received frame in a thread-safe way.
    with latest_frame_lock:
        latest_camera_frame = cv_image.copy()

    # TODO Display the image in an OpenCV window and wait 1 ms for OpenCV to process the GUI events.
    # This TODO is implemented in the main display loop instead of directly
    # inside the callback to prevent the OpenCV window from freezing.


# ---------------------------------------------------------------------------
# Block 4 - Main display loop.
# This loop keeps the OpenCV window responsive and shows the latest received
# frame at a controlled rate.
# ---------------------------------------------------------------------------
def display_camera_stream():
    global latest_camera_frame, camera_window_name

    cv2.namedWindow(camera_window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(camera_window_name, 960, 720)

    # This helps some OpenCV/Qt configurations inside Docker keep the window
    # event loop responsive.
    cv2.startWindowThread()

    display_rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        frame_to_show = None

        with latest_frame_lock:
            if latest_camera_frame is not None:
                frame_to_show = latest_camera_frame.copy()

        if frame_to_show is not None:
            cv2.imshow(camera_window_name, frame_to_show)

        key = cv2.waitKey(10) & 0xFF

        if key == 27 or key == ord("q"):
            rospy.loginfo("Closing robot camera viewer window.")
            rospy.signal_shutdown("User closed camera viewer.")
            break

        display_rate.sleep()

    cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# Block 5 - Main ROS node initialisation.
# The node subscribes to /camera/color/image_raw by default, as required by the
# assignment.
# ---------------------------------------------------------------------------
def main():
    rospy.init_node('show_camera_robot', anonymous=True)

    camera_image_topic = rospy.get_param(
        "~camera_image_topic",
        "/camera/color/image_raw"
    )

    rospy.loginfo("Starting robot camera viewer node.")
    rospy.loginfo("Subscribing to camera topic: %s", camera_image_topic)

    # TODO Subscribe to ROS topic that has the images
    rospy.Subscriber(
        camera_image_topic,
        Image,
        image_callback,
        queue_size=1,
        buff_size=2**24
    )

    display_camera_stream()


# ---------------------------------------------------------------------------
# Block 6 - Python entry point.
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    main()