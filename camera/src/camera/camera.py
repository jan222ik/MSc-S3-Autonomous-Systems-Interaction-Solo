#!/usr/bin/env python
from __future__ import print_function

import math

import sys
import rospy
import cv2
import tf
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from image_geometry.cameramodels import PinholeCameraModel
from tagstore.srv import TagstoreAddTag, TagstoreResetRVis
from transformations_odom.msg import PoseInMap
import numpy as np
from subprocess import call


class TagDetector:

    def __init__(self):
        rospy.init_node('tag_detector', anonymous=True)
        self.mapInfo = MapMetaData()
        self.mapInfo = rospy.wait_for_message("map", OccupancyGrid).info
        self.is_calculating = False
        self.image_pub = rospy.Publisher("/rupp/image_topic_tag", Image)
        # rospy.on_shutdown(self._shutdown)
        self.bridge = CvBridge()
        # self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.raspi_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.raspi_callback)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.img_raw_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        self.camera_rgb_info = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.cam_callback)
        self.cam = PinholeCameraModel()
        self.camera_info = None
        self.not_cam_setup = True
        self.depth_image = None
        self.pose = None
        self.saved_map = False
        self.cur_pose = None
        print("Setup done")

    def map_pose_callback(self, data):
        self.pose = (data.x, data.y)

    def odom_callback(self, data):
        point = data.pose.pose.position
        self.cur_pose = self.convert_point(point.x, point.y, 'odom', 'camera_link')

    def map_save_load_test(self, map_name, mode):
        rospy.wait_for_service("/map_save_load")
        try:
            map_save = rospy.ServiceProxy("/map_save_load", TagstoreAddTag)
            resp = map_save(map_name, mode)
            print("Saving map: ", resp)
        except rospy.ServiceException as e:
            print("Service call to save map failed: %s" % e)

    def add_tag_with_tagstore(self, x, y):
        rospy.wait_for_service("tagstore-addtag")
        try:
            tag_add = rospy.ServiceProxy("tagstore-addtag", TagstoreAddTag)
            resp = tag_add(x, y)
            print("Adding tag: ", resp)
        except rospy.ServiceException as e:
            print("Service call to add tag failed: %s" % e)

    def reset_tagstore(self):
        rospy.wait_for_service("tagstore-reset-rvis-markers")
        try:
            tag_reset = rospy.ServiceProxy("tagstore-reset-rvis-markers", TagstoreResetRVis)
            resp = tag_reset()
            print("Resetting markers: ", resp)
        except rospy.ServiceException as e:
            print("Service call to reset markers failed: %s" % e)


    def depth_callback(self, data):
        try:
            NewImg = self.bridge.imgmsg_to_cv2(data, "passthrough")
            self.depth_image = NewImg
            cv2.imshow("depth.png", NewImg)
        except CvBridgeError as e:
            print(e)

    def cam_callback(self, data):
        if self.not_cam_setup:
            self.camera_info = data
            self.cam.fromCameraInfo(data)
            self.not_cam_setup = False

    def convert_point(self, x, y, base_frame, target_frame):
        listener = tf.TransformListener()
        listener.waitForTransform(base_frame, target_frame, rospy.Time(0), rospy.Duration(4.0))
        stamp_point = PointStamped()
        stamp_point.header.frame_id = base_frame
        stamp_point.header.stamp = rospy.Time(0)
        stamp_point.point.x = x
        stamp_point.point.y = y
        stamp_point.point.z = 0.0
        return listener.transformPoint(target_frame, stamp_point)

    def raspi_callback(self, data):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        self.do_detection(cv_image)

    def img_raw_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.do_detection(cv_image)

    def do_detection(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # create a binary thresholded image on hue between red and yellow
        lower = (0, 100, 100)
        upper = (40, 255, 255)
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
        # cv2.imshow("TEST", cv_image)
        # print("TEST")
        for c in contours:
            cv2.drawContours(result1, [c], 0, (0, 0, 0), 2)
            # get first point of contour position
            x = c[0][0][0]
            y = c[0][0][1]
            print(self.mapInfo)

            # get depth at point
            depth_at_point = self.depth_image[x, y]

            # circle point in image (to check)
            cv2.circle(cv_image, (x, y), 20, (0, 255, 0))

            rect_point = self.cam.rectifyPoint((x, y))
            cam_ray = np.array(self.cam.projectPixelTo3dRay(rect_point))
            cam_point = cam_ray * depth_at_point

            # cur_point = (self.cur_pose.point.x + cam_point[0], self.cur_pose.point.y + cam_point[1])

            p = self.convert_point(cam_point[0], cam_point[1], "camera_link", "map")

            map_x = p.point.x
            map_y = p.point.y

            # TODO: not quite right, find error and fix!
            if self.mapInfo.resolution > 0:
                grid_x = ((map_x - self.mapInfo.origin.position.x) / self.mapInfo.resolution)
                grid_y = ((map_y - self.mapInfo.origin.position.y) / self.mapInfo.resolution)

                print("X: ", grid_x, ", Y: ", grid_y)

                self.add_tag_with_tagstore(int(grid_x) + 1, int(grid_y) + 1)
        #cv2.imshow("depth", self.depth_image)
        # rospy.sleep(10.0)
        cv2.imshow("image", cv_image)
        cv2.waitKey(3)

    def callback(self,data):
        if self.is_calculating:
            return
        else:
            self.is_calculating = True
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")


        except CvBridgeError as e:
            print(e)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # create a binary thresholded image on hue between red and yellow
        lower = (0, 100, 100)
        upper = (40, 255, 255)
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
        #print("TEST")
        for c in contours:
            cv2.drawContours(result1, [c], 0, (0, 0, 0), 2)
            # get first point of contour position
            x = c[0][0][0]
            y = c[0][0][1]

            # get depth at point
            #depth_at_point = self.depth_image[x, y]

            # circle point in image (to check)
            cv2.circle(cv_image, (x, y), 20, (0, 255, 0))

            #world_cord = np.array([x, y, 1])
            #camera_info_array = np.asarray(self.camera_info.K)
            #cam_reshaped = np.reshape(camera_info_array, (3, 3))

            #listener = tf.TransformListener()
            #(trans, rot) = listener.lookupTransform("/camera_link", "/map", rospy.Time(0))
            #print("WORLD CORD: ", world_cord)
            #world_cord = np.linalg.inv(cam_reshaped)*world_cord
            #world_cord *= depth_at_point
            #print("NEW WORLD CORD: ", world_cord)
            # rectify image and get unit vector from camera to pixel
            #rect_point = self.cam.rectifyPoint((x, y))
            #cam_ray = np.array(self.cam.projectPixelTo3dRay(rect_point))



            # multiply unit vector with depth to get vector from camera to point in image
            #cam_point = cam_ray * (depth_at_point) * 1000

            #p = self.convert_point(cam_point[0], cam_point[1], "camera_link", "base_link")
            #pp = self.convert_point(p.point.x, p.point.y, "base_link", "map")
            #print("OOOOHHHHH: ", pp)

            #test_x, test_y = self._robo_map_pose(cam_point[0], cam_point[1])
            # add vector to pose
            #map_cam_point_x = int(self.pose[0] + cam_point[0])
            #map_cam_point_y = int(self.pose[1] + cam_point[1])

            # add tag to tagstore
            #rospy.loginfo("NEW X AND Y: ", map_cam_point_x, ", ", map_cam_point_y)
            #rospy.logdebug("TEST X: ", world_cord[0], ", TEST Y: ", world_cord[1])
            # self.add_tag_with_tagstore(int(pp.point.x), int(pp.point.y))
            # further image stuff, not needed right now
            # get rotated rectangle from contour
            rot_rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rot_rect)
            box = np.int0(box)
            #print("BOX: ", box)
            # draw rotated rectangle on copy of img
            cv2.drawContours(result2, [box], 0, (0, 0, 0), 2)
            print("ADASSDas")

        # save result
        # cv2.imwrite("4cubes_thresh.jpg", thresh)
        # cv2.imwrite("4cubes_clean.jpg", clean)
        # cv2.imwrite("4cubes_result1.png", result1)
        # cv2.imwrite("4cubes_result2.png", result2)
        # show images (not sure if this or publish is correct)
        cv2.imshow("image", cv_image)
        cv2.imshow("thresh", thresh)
        cv2.imshow("clean", clean)
        cv2.imshow("result1", result1)
        cv2.imshow("result2", result2)
        self.is_calculating = False
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(thresh, "8UC1"))
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(clean, "8UC1"))
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(result1, "8UC3"))
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(result2, "8UC3"))
        except CvBridgeError as e:
            print(e)


def main(args):
    td = TagDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
