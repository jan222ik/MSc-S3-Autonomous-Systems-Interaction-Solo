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
from tf.transformations import euler_from_quaternion
from transformations_odom.msg import PoseInMap, PoseTF
from nav_msgs.msg import Odometry, OccupancyGrid

RATE = 0
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
        self.map_info = self.mapInfo = rospy.wait_for_message("map", OccupancyGrid).info
        self.setup_camera_seen_masks()
        self.robot_x_old = 0
        self.robot_y_old = 0
        self.robot_yaw_old = 0
        self.image_pub = rospy.Publisher("/image_topic_tag", Image, queue_size=1)
        self.vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.bridge = CvBridge()
        self.raspi_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.raspi_callback)
        self.cur_rob_pose = rospy.Subscriber("/pose_tf", PoseTF, self.pose_callback)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.img_raw_callback)
        self.rate_count = 0
        self.robot_pose = None
        self.turtle_yaw = 0
        self.turtle_x = 0
        self.turtle_y = 0
        self.turtle_covar = None
        self.pub_seen_map = rospy.Publisher('/camera/tag/costmap', OccupancyGrid, queue_size=1)
        self.pub_seen_middle = rospy.Publisher('/camera/tag/visible', PoseInMap, queue_size=1)
        self.map_camera_seen_initialised = False
        self.map_camera = [[]]
        self.map_camera_seen_seq = 0
        self.map_sub = rospy.Subscriber('map', OccupancyGrid, self._handle_map_update)
        self.last_img_pose = None
        self.is_calculating_image = False

        print("Setup done")

    def _calc_cov_area(self, cov_mat):
        """
        Calculate area of covariance ellipsis
        """
        # Cut off unneeded parts (since we cannot fly with the turtlebot, they are 0 anyway)
        cov_reduced = cov_mat[0:2, 0:2]

        # calculate eigenvectors/eigenvalues to get a and b
        eig_val, eig_vec = np.linalg.eig(np.linalg.inv(cov_reduced))
        eigen_x, eigen_y = eig_vec[:, 0]
        a, b = 2 / np.sqrt(eig_val)
        # area of ellipse is a * b * pi
        area = a * b * math.pi
        print("AREA OF ELLIPSE: ", str(area))
        return area

    def _publish_map_camera(self):
        """
        Publish map so it can be visualized in RViz and also used for path planing.
        """
        # create occupany grid to publish it
        oG = OccupancyGrid()
        # header
        oG.header.seq = self.map_camera_seen_seq
        oG.header.frame_id = "map"
        oG.header.stamp = rospy.Time.now()
        # set the info like it is in the original map
        oG.info = self.map_info
        # set the map reshaped as array
        if self.map_info is not None:
            oG.data = np.reshape(self.map_camera, self.map_info.height * self.map_info.width)

            self.pub_seen_map.publish(oG)

    def _handle_pose_update(self, data):
        """
        Handles the data that was published to the simple_odom_pose topic.
        """
        self.pose_converted = data.mapPose
        self.pose = data.originalPose

       # try:
            # update only when the pose changed
        if self.pose_converted.x != self.robot_x_old or self.pose_converted.y != self.robot_y_old or abs(
                self.pose_converted.angle - self.robot_yaw_old) > 0.1:
            # update the map
            self._update_map_camera_seen(self.pose_converted.x, self.pose_converted.y, self.pose_converted.angle)

            self.robot_x_old = self.pose_converted.x
            self.robot_y_old = self.pose_converted.y
            self.robot_yaw_old = self.pose_converted.angle
       # except:
        #    rospy.logerr('update map camera seen failed')


    def _handle_map_update(self, data):
        """
        Handles the data published to the map topic
        """
        self.map_info = data.info
        self.map = np.reshape(data.data, (data.info.height, data.info.width))

        # initialise the camera map if not done yet
        if not self.map_camera_seen_initialised:
            self.map_camera = np.full((data.info.height, data.info.width), -1)
            self._publish_map_camera()
            self.map_camera_seen_initialised = True

    def setup_camera_seen_masks(self):
        """
        Create the masks for setting the seen areas.
        """
        self.mask_camera_seen_north = np.array([[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                                [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                                [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                                [0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0],
                                                [0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0],
                                                [0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0],
                                                [0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0]])
        self.mask_camera_seen_north_offset_x = -6
        self.mask_camera_seen_north_offset_y = -9

        self.mask_camera_seen_north_east = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
                                                     [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
                                                     [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0],
                                                     [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                                     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
                                                     [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
                                                     [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        self.mask_camera_seen_north_east_offset_x = 0
        self.mask_camera_seen_north_east_offset_y = -14

        self.mask_camera_seen_east = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
                                               [0, 0, 0, 0, 0, 0, 1, 1, 1, 1],
                                               [0, 0, 0, 0, 1, 1, 1, 1, 1, 1],
                                               [0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
                                               [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                               [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                               [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                               [0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
                                               [0, 0, 0, 0, 1, 1, 1, 1, 1, 1],
                                               [0, 0, 0, 0, 0, 0, 1, 1, 1, 1],
                                               [0, 0, 0, 0, 0, 0, 0, 0, 1, 1]])
        self.mask_camera_seen_east_offset_x = 0
        self.mask_camera_seen_east_offset_y = -6

        self.mask_camera_seen_south_east = np.array([[1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
                                                     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
                                                     [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                                     [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0],
                                                     [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
                                                     [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        self.mask_camera_seen_south_east_offset_x = 0
        self.mask_camera_seen_south_east_offset_y = 0

        self.mask_camera_seen_south = np.array([[0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0],
                                                [0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0],
                                                [0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0],
                                                [0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0],
                                                [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                                [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                                [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]])
        self.mask_camera_seen_south_offset_x = -6
        self.mask_camera_seen_south_offset_y = 0

        self.mask_camera_seen_south_west = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1],
                                                     [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
                                                     [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                                     [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                     [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                     [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                                     [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        self.mask_camera_seen_south_west_offset_x = -14
        self.mask_camera_seen_south_west_offset_y = 0

        self.mask_camera_seen_west = np.array([[1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                               [1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
                                               [1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
                                               [1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                               [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                               [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                               [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                               [1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                               [1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
                                               [1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
                                               [1, 1, 0, 0, 0, 0, 0, 0, 0, 0]])
        self.mask_camera_seen_west_offset_x = -10
        self.mask_camera_seen_west_offset_y = -6

        self.mask_camera_seen_north_west = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                                     [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                                     [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                     [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                     [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                                     [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1]])
        self.mask_camera_seen_north_west_offset_x = -14
        self.mask_camera_seen_north_west_offset_y = -14

    def _map_camera_set_seen(self, x, y, mask, offset_x, offset_y):
        """
        This method sets the matrix of the map which represents the seen area by camera
        """
        x = int(x)
        y = int(y)
        offset_x = int(offset_x)
        offset_y = int(offset_y)
        y_mask = 0
        x_mask = 0

        y_start = y + offset_y
        y_end = (y + offset_y + len(mask))
        y_increment = 1

        x_start = x + offset_x
        x_end = (x + offset_x + len(mask[0]))
        x_increment = 1

        # rows
        for y1 in range(y_start, y_end, y_increment):
            x_mask = 0
            # cols
            for x1 in range(x_start, x_end, x_increment):
                # check that not outside of map
                if y1 >= 0 and x1 >= 0 and y1 < len(self.map_camera) and x1 < len(self.map_camera[0]):
                    if mask[y_mask][x_mask] == 1 and not self.map_camera[y1][x1] > 1:
                        self.map_camera[y1][x1] = 1
                        if self.last_img_pose is not None:
                            (ly, lx, angle, val) = self.last_img_pose
                            if y1 == ly and x1 == lx:
                                self.map_camera[y1][x1] = val
                                mp = PoseInMap()
                                mp.y = ly
                                mp.x = lx
                                mp.angle = angle
                                self.pub_seen_middle.publish(mp)

                x_mask = x_mask + 1

            y_mask = y_mask + 1

    def _update_map_camera_seen(self, x, y, yaw):
        """
        Updates the map that tracks the seen areas from the view of the camera
        """

        yaw = (yaw + math.pi / 2.0 + 2 * math.pi) % (2 * math.pi)

        # North
        if (yaw >= (15.0 / 16.0 * 2.0 * math.pi) and yaw < (2 * math.pi)) or (
                yaw >= 0 and yaw < (1.0 / 16.0 * 2.0 * math.pi)):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_north, self.mask_camera_seen_north_offset_x,
                                      self.mask_camera_seen_north_offset_y)

        # North East
        if yaw >= (1.0 / 16.0 * 2.0 * math.pi) and yaw < (3.0 / 16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_north_east, self.mask_camera_seen_north_east_offset_x,
                                      self.mask_camera_seen_north_east_offset_y)

        # East
        if yaw >= (3.0 / 16.0 * 2.0 * math.pi) and yaw < (5.0 / 16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_east, self.mask_camera_seen_east_offset_x,
                                      self.mask_camera_seen_east_offset_y)

        # South East
        if yaw >= (5.0 / 16.0 * 2.0 * math.pi) and yaw < (7.0 / 16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_south_east, self.mask_camera_seen_south_east_offset_x,
                                      self.mask_camera_seen_south_east_offset_y)

        # South
        if yaw >= (7.0 / 16.0 * 2.0 * math.pi) and yaw < (9.0 / 16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_south, self.mask_camera_seen_south_offset_x,
                                      self.mask_camera_seen_south_offset_y)

        # South West
        if yaw >= (9.0 / 16.0 * 2.0 * math.pi) and yaw < (11.0 / 16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_south_west, self.mask_camera_seen_south_west_offset_x,
                                      self.mask_camera_seen_south_west_offset_y)

        # West
        if yaw >= (11.0 / 16.0 * 2.0 * math.pi) and yaw < (13.0 / 16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_west, self.mask_camera_seen_west_offset_x,
                                      self.mask_camera_seen_west_offset_y)

        # North West
        if yaw >= (13.0 / 16.0 * 2.0 * math.pi) and yaw < (15.0 / 16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_north_west, self.mask_camera_seen_north_west_offset_x,
                                      self.mask_camera_seen_north_west_offset_y)


        self.map_camera_seen_seq = self.map_camera_seen_seq + 1

        self._publish_map_camera()

    def odom_callback(self, odometry):
        pose = odometry.pose
        covar = pose.covariance
        pose_pose = pose.pose
        point = pose_pose.position
        quat = pose_pose.orientation
       # (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
       # self.turtle_yaw = yaw
       # self.turtle_x = (point.x - self.mapInfo.origin.position.x) /  self.map_info.resolution
       # self.turtle_y = (point.y - self.mapInfo.origin.position.y) / self.map_info.resolution
        cov_arr = np.asarray(covar)
        cov_mat = np.reshape(cov_arr, (6, 6))
        self.turtle_covar = cov_mat

    def pose_callback(self, data):
        self.robot_pose = data.mapPose
        self._handle_pose_update(data)

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
        if not self.is_calculating_image:
            self.is_calculating_image = True
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

                            if self.robot_pose is not None:
                                # get robot angel and rotate
                                rot_point = self.rotate(flipped_origin, flipped_cen, self.robot_pose.angle)
                                # calculate map pos
                                # TODO: determine proper scaling
                                map_y = self.robot_pose.y + rot_point[0]//20
                                map_x = self.robot_pose.x + (-1 * rot_point[1]//20)  # x direction is flipped in map
                                map_x = int(map_x)
                                map_y = int(map_y)
                                area = self._calc_cov_area(self.turtle_covar)
                                val = -1
                                if area <= 0.00001:
                                    val = 100
                                elif area <= 0.0001:
                                    val = 80
                                elif area <= 0.001:
                                    val = 60
                                elif area <= 0.1:
                                    val = 50
                                self.last_img_pose = (map_x, self.robot_pose.angle, map_y, val)
                                #self._publish_map_camera()

                                #print("TAG POS IN MAP: Y: ", map_y, ", X: ", map_x)
                                # TODO: works in principle, but uncertainty leads to tag overload, fix this
                                #self.add_tag_with_tagstore(int(map_y), int(map_x))

                cv2.imshow("image", cv_image)
                self.rate_count = 0
                cv2.waitKey(3)
                self.is_calculating_image = False
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
