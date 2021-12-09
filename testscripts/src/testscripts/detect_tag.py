#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from image_geometry.cameramodels import PinholeCameraModel
import numpy as np

class tag_detector:

  def __init__(self):
    self.image_pub = rospy.Publisher("/rupp/image_topic_tag",Image)
    # self.image_grey_pub = rospy.Publisher("/rupp/image_topic_grey",Image)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
    #self.camera_param_desc = rospy.Subscriber("/camera/parameter_descriptions", self.param_callback)
    self.camera_rgb_info = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.cam_callback)
    self.cam = PinholeCameraModel()
    self.not_cam_setup = True

  def cam_callback(self, data):
    if self.not_cam_setup:
      self.cam.fromCameraInfo(data)
      self.not_cam_setup = False

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")


    except CvBridgeError as e:
      print(e)

    # cv2.imshow("TEST", cv_image)
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # create a binary thresholded image on hue between red and yellow
    lower = (0, 240, 160)
    upper = (30, 255, 255)
    thresh = cv2.inRange(hsv, lower, upper)

    # apply morphology
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    clean = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    clean = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    # get external contours
    contours = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]

    result1 = cv_image.copy()
    result2 = cv_image.copy()
    #cv2.imshow("TEST", cv_image)
    for c in contours:
      cv2.drawContours(result1, [c], 0, (0, 0, 0), 2)
      print(c[:][0][0])
      x = c[0][0][0]
      y = c[0][0][1]
      cv2.circle(cv_image, (x, y), 20, (0, 255, 0))
      rect_point = self.cam.rectifyPoint((x, y))
      cam_point = self.cam.projectPixelTo3dRay(rect_point)
      print((x, y), ", with image_geometry: ", cam_point)
      # get rotated rectangle from contour
      rot_rect = cv2.minAreaRect(c)
      box = cv2.boxPoints(rot_rect)
      box = np.int0(box)
      #print("BOX: ", box)
      # draw rotated rectangle on copy of img
      cv2.drawContours(result2, [box], 0, (0, 0, 0), 2)

    # save result
    # cv2.imwrite("4cubes_thresh.jpg", thresh)
    # cv2.imwrite("4cubes_clean.jpg", clean)
    # cv2.imwrite("4cubes_result1.png", result1)
    # cv2.imwrite("4cubes_result2.png", result2)
    cv2.imshow("image", cv_image)
    #cv2.imshow("thresh", thresh)
    #cv2.imshow("clean", clean)
    cv2.imshow("result1", result1)
    #cv2.imshow("result2", result2)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      #self.image_pub.publish(self.bridge.cv2_to_imgmsg(thresh, "8UC1"))
      #self.image_pub.publish(self.bridge.cv2_to_imgmsg(clean, "8UC1"))
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(result1, "8UC3"))
      #self.image_pub.publish(self.bridge.cv2_to_imgmsg(result2, "8UC3"))
    except CvBridgeError as e:
      print(e)

def main(args):
  td = tag_detector()
  rospy.init_node('tag_detector', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)