#!/usr/bin/python3

import rospy
import cv2
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from colorama import Fore, Style
#from model import KeyPointClassifier
def publish_image():

    bridge = CvBridge()

    try:
        camera1_name = rospy.get_param("my_webcam/camera_hand_pose")
        print(Fore.BLUE + "camera1_name: %s" + Style.RESET_ALL, camera1_name)
        image_pub1 = rospy.Publisher("raw_image", Image, queue_size=10)
        capture1 = cv2.VideoCapture(camera1_name)
    
    except Exception as e:
        print(Fore.RED + f"Error: {e}"+ Style.RESET_ALL)
        capture1 = None  # Marcar la cámara como no disponible

    try:
        camera2_name = rospy.get_param("my_webcam/camera_yolo")
        print(Fore.BLUE + "camera2_name: %s" + Style.RESET_ALL, camera2_name)
        image_pub2 = rospy.Publisher("raw_image", Image, queue_size=10)
        capture2 = cv2.VideoCapture(camera2_name)
    except Exception as e:
        print(Fore.RED + f"Error: {e}"+ Style.RESET_ALL)
        capture2 = None  # Marcar la cámara como no disponible

    rospy.init_node('img_publisher', anonymous=False)
    rate = rospy.Rate(100)  # Incrementa la tasa a 100 Hz (ajusta según sea necesario)

    while not rospy.is_shutdown():
        if capture1 is not None:
            ret1 , img1 = capture1.read()
            if ret1 is True:
                pose_message1 = bridge.cv2_to_imgmsg(img1, "bgr8")
                image_pub1.publish(pose_message1)
        #        cv2.imshow('Camera 1 Image', img1)

        if capture2 is not None:
            ret2 , img2 = capture2.read()
            if ret2 is True:
                pose_message2 = bridge.cv2_to_imgmsg(img2, "bgr8")
                image_pub2.publish(pose_message2)
  #              cv2.imshow('Camera 2 Image', img2)
#                cv2.imshow(img2)
        rate.sleep()


if __name__ == '__main__':
    try:
        publish_image()
    except rospy.ROSInterruptException as e:
        print(Fore.RED + f"Error: {e}" + Style.RESET_ALL)