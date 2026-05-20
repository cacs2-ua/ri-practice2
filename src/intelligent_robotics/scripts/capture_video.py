#!/usr/bin/env python3

# ---------------------------------------------------------------------------
# Block 1 - Import required libraries.
# This ROS node obtains the operator image from a webcam or video file using
# OpenCV and publishes it as a ROS Image message in /operator/image.
# ---------------------------------------------------------------------------
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# ---------------------------------------------------------------------------
# Block 2 - Main video publisher function.
# This function keeps the original structure of the initial file:
#   1. Initialise the ROS node.
#   2. Create the /operator/image publisher.
#   3. Open a webcam or a video file.
#   4. Capture frames.
#   5. Convert them to ROS Image messages.
#   6. Publish them continuously.
# ---------------------------------------------------------------------------
def video_publisher():
    # Initialise a ROS node
    rospy.init_node('video_publisher', anonymous=True)

    # Runtime parameters. These allow testing different webcams or video files
    # without modifying the source code.
    operator_image_topic = rospy.get_param("~operator_image_topic", "/operator/image")
    camera_index = rospy.get_param("~camera_index", 0)
    video_file = rospy.get_param("~video_file", "")
    publish_rate_hz = rospy.get_param("~publish_rate", 30)
    display_preview = rospy.get_param("~display_preview", True)
    flip_operator_image = rospy.get_param("~flip_operator_image", True)

    rospy.loginfo("Starting operator video publisher.")
    rospy.loginfo("Publishing operator images on topic: %s", operator_image_topic)

    # TODO Create a publisher in the /operator/image topic.
    operator_image_publisher = rospy.Publisher(
        operator_image_topic,
        Image,
        queue_size=1
    )

    # TODO Set up video capture from webcam (or from video)
    if video_file:
        rospy.loginfo("Opening operator video file: %s", video_file)
        cap = cv2.VideoCapture(video_file)
    else:
        camera_index = int(camera_index)
        rospy.loginfo("Opening operator webcam with camera index: %d", camera_index)
        cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        rospy.logerr("Could not open the webcam/video source.")
        rospy.logerr("Check /dev/video*, Docker device permissions, or the _camera_index parameter.")
        return

    # Configure a reasonable image size and frame rate for real-time gesture
    # recognition without making the system unnecessarily heavy.
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, publish_rate_hz)

    # Create an instance of CvBridge to convert OpenCV images to ROS messages.
    bridge = CvBridge()

    # Define the publication rate (e.g., 10 Hz).
    rate = rospy.Rate(publish_rate_hz)

    while not rospy.is_shutdown():
        # TODO Capture a frame from the webcam
        frame_was_read, operator_frame = cap.read()

        if not frame_was_read:
            rospy.logwarn_throttle(2.0, "Could not read frame from webcam/video source.")

            # If a video file is being used, restart it when it reaches the end.
            if video_file:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

            rate.sleep()
            continue

        # Mirror the image so gestures feel natural to the operator.
        # This can be disabled with _flip_operator_image:=false.
        if flip_operator_image:
            operator_frame = cv2.flip(operator_frame, 1)

        # TODO Convert OpenCV frame to ROS message
        operator_image_message = bridge.cv2_to_imgmsg(operator_frame, encoding="bgr8")
        operator_image_message.header.stamp = rospy.Time.now()
        operator_image_message.header.frame_id = "operator_camera"

        # TODO Post the message in the topic
        operator_image_publisher.publish(operator_image_message)

        # Optional preview window for debugging the webcam capture.
        if display_preview:
            cv2.imshow("Operator Camera", operator_frame)
            key = cv2.waitKey(1) & 0xFF

            if key == 27 or key == ord("q"):
                rospy.loginfo("Closing operator video publisher.")
                rospy.signal_shutdown("Operator closed the preview window.")

        # Waiting to meet the publication rate
        rate.sleep()

    # When you're done, release the catch
    cap.release()
    cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# Block 3 - Python entry point.
# This preserves the original executable structure of the initial file.
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        video_publisher()
    except rospy.ROSInterruptException:
        pass