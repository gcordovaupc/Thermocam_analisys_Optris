#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('Bag_OpenCV')
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rosbag
from sensor_msgs.msg import Image
#from sensor_msgs.msg import Image as UInt8
import numpy as np
bridge = CvBridge()


def nothing(x):
    pass  
    
cv2.namedWindow("Trackbars")
cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic",Image, queue_size=10)
#    self.data_pub = rospy.Publisher("theora_topic",ResizedImage, queue_size=10) #check how to add them to the image

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/thermal_image_view",Image,self.callback)
#    self.data_sub = rospy.Subscriber("/thermal_image_view/theora",ResizedImage,self.callback)
  
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

#    (rows,cols,channels) = cv_image.shape
#    if cols > 70 and rows > 60 :
#      cv2.circle(cv_image, (50,50), 30, 255)
    
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    
    l_h = cv2.getTrackbarPos("L - H", "Trackbars")
    l_s = cv2.getTrackbarPos("L - S", "Trackbars")
    l_v = cv2.getTrackbarPos("L - V", "Trackbars")
    u_h = cv2.getTrackbarPos("U - H", "Trackbars")
    u_s = cv2.getTrackbarPos("U - S", "Trackbars")
    u_v = cv2.getTrackbarPos("U - V", "Trackbars")
    
    lower_blue = np.array([l_h, l_s, l_v])
    upper_blue = np.array([u_h, u_s, u_v])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    result = cv2.bitwise_and(cv_image, cv_image, mask=mask)
    
    cv2.namedWindow("BGR image", cv2.WINDOW_NORMAL)
    cv2.imshow("BGR image", cv_image)
#    cv2.namedWindow("HSV image", cv2.WINDOW_NORMAL)
#    cv2.imshow("HSV",hsv)
    cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
    cv2.imshow("Mask", mask)
    cv2.namedWindow("Filter image", cv2.WINDOW_NORMAL)
    cv2.imshow("Filter image", result)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
