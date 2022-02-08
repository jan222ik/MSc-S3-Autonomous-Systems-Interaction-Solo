#!/usr/bin/env python

from __future__ import print_function

import rospy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
import numpy as np
import math
from std_srvs.srv import Empty
import actionlib
from localization.msg import LocalizationState, LocalizationAction, LocalizationResult, LocalizationFeedback
from sensor_msgs.msg import LaserScan
from tf import transformations


class WallFollowerTwo:

    def __init__(self):
        self.rate = rospy.Rate(20)
        self.state = 0
        self.botVelPub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laserSub = rospy.Subscriber('/scan', LaserScan, self.updateLaserData)
        self.laserData = None

    def updateLaserData(self, msg):
        self.laserData = msg.ranges

    def step(self):
        twist = self.sensePlan()
        self.botVelPub.publish(twist)

        self.rate.sleep()

    def sensePlan(self):
        if not self.laserData:
            print("Laser data not present")
            return Twist()

        data = np.array(self.laserData)
        k = 5
        closestK = np.argpartition(data, k)
        print(closestK[:k])
        #  print(data[closestK[:k]])

        d = 0.5  # Distance to keep away from wall
        twist = Twist()

        if data[0] < d:
            # Frontal obstacle detected -> avoidance
            print("Evade front - Turn Left")
            twist.angular.z = 0.2
        elif 260 < closestK[0] < 280:
            # The closest measurement is to the right side
            print("Follow Wall")
            twist.linear.x = 0.2
            # Correct steering
            steer = max(min(closestK[0] - 270, 1), -1)
            if data[closestK[0]] < (d / 1.5):
                # Override steer if entered distance threshold
                steer = 1
            twist.angular.z = 0.005 * steer
        elif not (120 < closestK[0] < 240) and data[closestK[0]] < d:
            # Obstacle ahead, need to turn
            # Fast rotate based on sensor provided angle
            if 270 < closestK[0] or closestK[0] < 90:
                print("Turn Left")
                twist.angular.z = 0.2
            else:
                print("Turn Right")
                twist.angular.z = -0.05
        else:
            # Dive to wall or clear back of robot if closest walls are behind
            if not (120 < closestK[0] < 240):
                print("Lost wall")
            else:
                print("Forward to clear back of robot")
            twist.linear.x = 0.05
        return twist


class Localizer:
    def __init__(self):
        # Subscribe to amcl to get localization PoseWithCovariance
        self.pose_estimate = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._pose_estimate_callback)

        # get service for global localization
        rospy.wait_for_service('global_localization')
        self.global_localisation = rospy.ServiceProxy('global_localization', Empty)

        # Set up action server (result, feedback, server) and start it
        self.loc_feedback = LocalizationFeedback()
        self.loc_result = LocalizationResult()
        self.loc_action = actionlib.SimpleActionServer(name='localize_me',
                                                       ActionSpec=LocalizationAction,
                                                       execute_cb=self._locate_me,
                                                       auto_start=False)

    def _pose_estimate_callback(self, estimated_pose):
        # Deconstructing PoseWithCovarianceStamped
        ## Header
        header = estimated_pose.header
        seq = header.seq
        time_stamp = header.stamp
        frame_id = header.frame_id

        ## Pose with covariance
        pose_with_cov = estimated_pose.pose
        self.loc_feedback.cur_loc_state.pose_w_cov = pose_with_cov

        ### Pose
        pose = pose_with_cov.pose

        #### Point
        point = pose.position
        point_x = point.x
        point_y = point.y
        point_z = point.z

        #### Quaternion
        quaternion = pose.orientation
        quat_x = quaternion.x
        quat_y = quaternion.y
        quat_z = quaternion.z
        quat_w = quaternion.w

        ### Covariance (float64[36])
        covariance = pose_with_cov.covariance
        cov_arr = np.asarray(covariance)
        cov_mat = np.reshape(cov_arr, (6, 6))
        self.loc_feedback.cur_loc_state.ellipse_area = self._calc_cov_area(cov_mat)

    def _locate_me(self, goal):
        # perform global localization once at the beginning
        # note: for small maps with few/no obstacles, this is actually counterproductive!
        self.global_localisation()
        # helper variables
        r = rospy.Rate(1)
        success = True
        # Initialize locate message
        self.loc_feedback.cur_loc_state.epsilon = goal.epsilon
        self.loc_feedback.cur_loc_state.ellipse_area = np.Inf
        wall_follow = WallFollowerTwo()
        while self.loc_feedback.cur_loc_state.ellipse_area > goal.epsilon:
            wall_follow.step()
            if self.loc_action.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % 'localize_me')
                self.loc_action.set_preempted()
                success = False
                break
            self.loc_action.publish_feedback(self.loc_feedback)
        if success:
            self.loc_result.loc_end_state = self.loc_feedback.cur_loc_state
            rospy.loginfo('%s: Succeeded' % 'localize_me')
            self.loc_action.set_succeeded(self.loc_result)

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

def main(args):
    loc = Localizer()
    rospy.init_node('localize_robot', anonymous=True)
    loc.loc_action.start()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
