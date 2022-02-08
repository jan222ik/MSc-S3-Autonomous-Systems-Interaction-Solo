#!/usr/bin/env python
from __future__ import print_function

import math

import sys
import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from tagstore.srv import TagstoreAddTag, TagstoreResetRVis
import numpy as np
from transformations_odom.msg import PoseInMap, PoseTF

RATE = 4
# HSV (Hue, Saturation, V)
# H : (0, 10), S: (200, 255), V: (20, 255)
#lab
#HSV_LOWER = (150, 100, 20)
#HSV_UPPER = (180, 255, 255)
# sim
HSV_LOWER = (0, 150, 150)
HSV_UPPER = (40, 255, 255)

class TagsToMap:
    """
    Node for converting red contours detected in image to tags in map.
    """

    def __init__(self):
        rospy.init_node('tags_to_map', anonymous=True)
        self.image_pub = rospy.Publisher("/image_topic_tag", Image, queue_size=1)
        self.vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.bridge = CvBridge()
        self.raspi_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.raspi_callback)
        self.cur_rob_pose = rospy.Subscriber("/pose_tf", PoseTF, self.pose_callback)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.img_raw_callback)
        self.rate_count = 0
        self.robot_pose = None
        print("Setup done")

    def pose_callback(self, data):
        self.robot_pose = data.mapPose

    def add_tag_with_tagstore(self, x, y):
        rospy.wait_for_service("tagstore-addtag")
        try:
            tag_add = rospy.ServiceProxy("tagstore-addtag", TagstoreAddTag)
            resp = tag_add(x, y)
            print("Adding tag: ", resp)
        except rospy.ServiceException as e:
            print("Service call to add tag failed: %s" % e)

    def raspi_callback(self, data):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        self.do_detection(cv_image)

    def img_raw_callback(self, data):
        print("ROWS: ", data.width, ", COLS: ", data.height)
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.do_detection(cv_image)

    def rotate(self, origin, point, angle):
        """
        Rotate a point counterclockwise by a given angle around a given origin.

        The angle should be given in radians.
        """
        oy, ox = origin
        py, px = point

        qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
        qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
        return qy, qx

    def do_detection(self, cv_image):
        print("DETECTION_COUNT: ", self.rate_count)
        rate = rospy.get_param('/cam_rate', 4)
        if self.rate_count >= rate:
            print("DOING DETECTION!!!")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # create a binary thresholded image on hue
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
            center_x = w//2
            center_y = h//2
            # loop over contours
            if len(contours) == 0:
                return
            else:
                for c in contours:
                    # compute the center of the contour
                    M = cv2.moments(c)
                    if M["m00"] != 0.0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])

                        # draw the contour and center of the shape on the image
                        cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 2)
                        cv2.circle(cv_image, (cX, cY), 7, (255, 255, 255), -1)
                        cv2.putText(cv_image, "center", (cX - 20, cY - 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        cv2.circle(cv_image, (center_x, h), 7, (0, 0, 255), -1)
                        cv2.putText(cv_image, "center_image", (center_x - 20, h - 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        cv2.line(cv_image, (center_x, h), (cX, cY), (255, 0, 0), 2)

                        # TODO: remove for real sim runs
                        center_x = cX  # test for simulation, since camera is not aligned properly there

                        # origin is in camera center, move other point
                        new_origin = (h - h, center_x - center_x)
                        new_cen = (cY - h, cX - center_x)

                        # flip y values
                        flipped_origin = (new_origin[0] * -1, new_origin[1])
                        flipped_cen = (new_cen[0] * -1, new_cen[1])

                        # calculate vector
                        vec = (flipped_cen[0] - flipped_origin[0], flipped_cen[1] - flipped_origin[1])

                        # calculate line length
                        new_line_len = np.linalg.norm((vec[0], vec[1]))
                        # calulcate angle between origin and center
                        new_line_angle = (math.atan2(vec[0], vec[1]) * 180) / math.pi
                        #new_line_angle = int((math.atan2((flipped_origin[1] - flipped_cen[1]), (flipped_origin[0] - flipped_cen[0])) * 180 / math.pi))
                        if new_line_angle < 0:
                            # new_line_angle = 360 + new_line_angle
                            print("HOW DID THIS HAPPEN?!")
                        print("NEW LINE LENGTH: ", new_line_len)
                        print("NEW LINE ANGLE: ", new_line_angle)

                        if self.robot_pose is not None:
                            # get robot angel and rotate
                            rot_point = self.rotate(flipped_origin, flipped_cen, self.robot_pose.angle)
                            # calculate map pos
                            # TODO: determine proper scaling
                            map_y = self.robot_pose.y + rot_point[0]//20
                            map_x = self.robot_pose.x + (-1 * rot_point[1]//20)  # x direction is flipped in map
                            print("TAG POS IN MAP: Y: ", map_y, ", X: ", map_x)
                            # TODO: works in principle, but uncertainty leads to tag overload, fix this
                            self.add_tag_with_tagstore(int(map_y), int(map_x))

            cv2.imshow("image", cv_image)
            self.rate_count = 0
            cv2.waitKey(3)
            # self.isActive = False
        else:
            self.rate_count += 1


def main(args):
    td = TagsToMap()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
