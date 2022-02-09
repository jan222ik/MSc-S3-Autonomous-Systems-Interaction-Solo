#!/usr/bin/env python
from __future__ import print_function

import math

import sys
import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np

RATE = 4
# HSV (Hue, Saturation, V)
# H : (0, 10), S: (200, 255), V: (20, 255)
# lab
#HSV_LOWER = (150, 100, 20)
#HSV_UPPER = (180, 255, 255)


# sim
HSV_LOWER = (0, 150, 150)
HSV_UPPER = (40, 255, 255)

class GoToCam:
    """
    Node for driving towards red contours detected in image.
    """

    def __init__(self):
        rospy.init_node('goto_camera', anonymous=True)
        self.image_pub = rospy.Publisher("/image_topic_tag", Image)
        self.vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.bridge = CvBridge()
        self.raspi_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.raspi_callback)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.img_raw_callback)
        self.rate_count = 0
        print("Setup done")

    def turn_left(self):
        twist = Twist()
        twist.linear.x = 0.05
        twist.angular.z = 0.1
        self.vel.publish(twist)

    def turn_right(self):
        twist = Twist()
        twist.linear.x = 0.05
        twist.angular.z = -0.1
        self.vel.publish(twist)

    def raspi_callback(self, data):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        self.do_detection(cv_image)

    def img_raw_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.do_detection(cv_image)

    def do_detection(self, cv_image):
        print("DETECTION_COUNT: ", self.rate_count)
        rate = rospy.get_param('/cam_rate', 4)
        if self.rate_count >= rate:
            print("DOING DETECTION!!!")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # create a binary thresholded image on hue between red and yellow
            lower_hsv = (rospy.get_param('lower_h', 0),
                         rospy.get_param('lower_s', 150),
                         rospy.get_param('lower_v', 150))
            upper_hsv = (rospy.get_param('upper_h', 40),
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
            center_x = w // 2
            center_y = h // 2
            # loop over contours
            if len(contours) == 0:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.vel.publish(twist)
                return
            else:
                for c in contours:
                    # compute the center of the contour
                    M = cv2.moments(c)
                    if M["m00"] != 0.0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        center_arr.append((cX, cY))
                        c_y = cv_image.shape[0] // 2
                        c_x = cv_image.shape[1] // 2
                        # draw the contour and center of the shape on the image
                        cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 2)
                        cv2.circle(cv_image, (cX, cY), 7, (255, 255, 255), -1)
                        cv2.putText(cv_image, "center", (cX - 20, cY - 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        cv2.circle(cv_image, (center_x, h), 7, (0, 0, 255), -1)
                        cv2.putText(cv_image, "center_image", (center_x - 20, h - 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        cv2.line(cv_image, (center_x, h), (cX, cY), (255, 0, 0), 2)
                        # print("cx: ", cX, ", cy: ", cY, ", center_x: ", center_x, ", center_y: ", center_y)
                        # Vektor: Spitze - Schaft in Form (y, x)
                        vec_to_tag = (cY - h, cX - center_x)
                        print("VEKTOR: ", vec_to_tag)
                        line_len = np.linalg.norm(np.array((cX - center_x, cY - h)))
                        line_angle = int((math.atan2((h - cY), (center_x - cX)) * 180 / math.pi))
                        print("LINE_LENGTH: ", line_len)
                        print("LINE_ANGLE: ", line_angle)
                        # origin is in camera center, move other point
                        new_origin = (h - h, center_x - center_x)
                        new_cen = (cY - h, cX - center_x)
                        # print("NEW POINT: ", new_cen, "; ORIGIN: ", new_origin)
                        # flip y values
                        flipped_origin = (new_origin[0] * -1, new_origin[1])
                        flipped_cen = (new_cen[0] * -1, new_cen[1])
                        print("FLIPPED POINT: ", flipped_cen, ", FLIPPED ORIGIN: ", flipped_origin)
                        new_line_len = np.linalg.norm(
                            np.array((flipped_cen[1] - flipped_origin[1], flipped_cen[0] - flipped_origin[0])))
                        new_line_angle = int((math.atan2((flipped_origin[1] - flipped_cen[1]),
                                                         (flipped_origin[0] - flipped_cen[0])) * 180 / math.pi))
                        print("NEW LINE LENGTH: ", new_line_len)
                        print("NEW LINE ANGLE: ", new_line_angle)
                        if new_line_angle < 0:
                            new_line_angle = 360 + new_line_angle

                        if new_line_angle > 90:
                            self.turn_left()
                        elif new_line_angle < 90:
                            self.turn_right()
                        else:
                            twist = Twist()
                            twist.linear.x = 0.1
                            twist.angular.z = 0.0
                            self.vel.publish(twist)

            cv2.imshow("image", cv_image)
            self.rate_count = 0
            cv2.waitKey(3)
        else:
            self.rate_count += 1


def main(args):
    goto_cam = GoToCam()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
