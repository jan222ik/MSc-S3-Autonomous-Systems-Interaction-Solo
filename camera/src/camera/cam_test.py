#!/usr/bin/env python
from __future__ import print_function

import math

import sys
import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

RATE = 0
# HSV (Hue, Saturation, V)
# H : (0, 10), S: (200, 255), V: (20, 255)
#lab
HSV_LOWER = (150, 100, 20)
HSV_UPPER = (180, 255, 255)
# sim
#HSV_LOWER = (0, 150, 150)
#HSV_UPPER = (40, 255, 255)

class CameraTest:
    """
    Node for converting red contours detected in image to tags in map.
    """

    def __init__(self):
        rospy.init_node('test_camera', anonymous=True)
        self.image_pub = rospy.Publisher("/image_topic_tag", Image, queue_size=1)
        self.bridge = CvBridge()
        self.raspi_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.raspi_callback)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.img_raw_callback)


        print("Setup done")

    def raspi_callback(self, data):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        self.do_detection(cv_image)

    def img_raw_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.do_detection(cv_image)

    def do_detection(self, cv_image):
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # create a binary thresholded image on hue
            lower_hsv = (rospy.get_param('lower_h', 150),
                         rospy.get_param('lower_s', 100),
                         rospy.get_param('lower_v', 20))
            upper_hsv = (rospy.get_param('upper_h', 180),
                         rospy.get_param('upper_s', 255),
                         rospy.get_param('upper_v', 255))
            lower = lower_hsv
            upper = upper_hsv
            thresh = cv2.inRange(hsv, lower, upper)

            # apply morphology
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
            clean = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
            clean = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

            # get external contours
            contours = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = contours[0] if len(contours) == 2 else contours[1]

            center_arr = []
            (h, w) = cv_image.shape[:2]  # w:image-width and h:image-height
            center_x = w//2
            center_y = h//2
            # loop over contours
            if len(contours) == 0:
                return
            else:
                for c in contours:
                    M = cv2.moments(c)
                    if M["m00"] != 0.0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        rospy.loginfo("FOUND TAG")

                        # draw the contour and center of the shape on the image
                        cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 2)
                        cv2.circle(cv_image, (cX, cY), 7, (255, 255, 255), -1)
                        cv2.putText(cv_image, "center", (cX - 20, cY - 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        cv2.circle(cv_image, (center_x, h), 7, (0, 0, 255), -1)
                        cv2.putText(cv_image, "center_image", (center_x - 20, h - 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        cv2.line(cv_image, (center_x, h), (cX, cY), (255, 0, 0), 2)

            cv2.imshow("image", cv_image)
            cv2.waitKey(3)


def main(args):
    test = CameraTest()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
