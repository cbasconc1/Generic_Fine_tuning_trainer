#!/usr/bin/python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def callback(image_msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_msg)
    cv2.imshow('ROS Image Subscriber 1', cv_image)
    cv2.waitKey(1)  # Wait for 1 millisecond instead of 1000 (no delay)

if __name__ == '__main__':
    rospy.init_node("image_subscriber", anonymous=False)
    print("Subscribe images from topic /image_raw_alejandro_2 ...")

    image_subcriber2 = rospy.Subscriber("image_raw_alejandro_2", Image, callback)

    try:
        # spin() simply keeps python from exiting until this node is stopped
        
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!")
        cv2.destroyAllWindows()  # Close OpenCV windows before shutting down
