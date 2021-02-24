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



###### This program is to test the record vessel_sample2.bag which


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
    
     
    lower_blue_noz = np.array([16,0,117])
    upper_blue_noz = np.array([119,255,255])
    
    lower_blue_dot = np.array([3,0,0])
    upper_blue_dot = np.array([119,255,255])
    
    nozzle = cv2.inRange(hsv, lower_blue_noz, upper_blue_noz)
    dots = cv2.inRange(hsv, lower_blue_dot, upper_blue_dot)
    
    remove = dots - nozzle
#    remove = cv2.absdiff(nozzle,dots)
#    remove = cv2.subtract(dots,nozzle) 
#    mask = dots - nozzle
    result = cv2.bitwise_and(cv_image, cv_image,mask=remove)
    
    cv2.namedWindow("BGR original", cv2.WINDOW_NORMAL)
    cv2.imshow("BGR original", cv_image)
    cv2.namedWindow("Nozzle", cv2.WINDOW_NORMAL)
    cv2.imshow("Nozzle",nozzle)
    cv2.namedWindow("Dots", cv2.WINDOW_NORMAL)
    cv2.imshow("Dots",dots)
    
    
    cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
    cv2.imshow("Mask", remove)
    
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
