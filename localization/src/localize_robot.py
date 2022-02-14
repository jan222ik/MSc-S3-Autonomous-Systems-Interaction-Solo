#!/usr/bin/env python

from __future__ import print_function

import rospy
import sys
from operator import itemgetter
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
import numpy as np
import math
from std_srvs.srv import Empty
import actionlib
from localization.msg import LocalizationState, LocalizationAction, LocalizationResult, LocalizationFeedback
from sensor_msgs.msg import LaserScan
from tf import transformations


LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR


# code from turtlebot3_examples -> nodes -> turtlebot3_obstacle
class Obstacle:
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        return scan.ranges

    def wall_follow(self):
        twist = Twist()
        turtlebot_moving = True

        lidar_distances = self.get_scan()
        min_distance = min(lidar_distances)
        min_index = min(enumerate(lidar_distances), key=itemgetter(1))[0]
        angular = 0.0
        linear = LINEAR_VEL
        if min_distance < SAFE_STOP_DISTANCE * 2:
            if min(lidar_distances[:10]) < SAFE_STOP_DISTANCE or min(lidar_distances[354:364]) < SAFE_STOP_DISTANCE:
                angular = math.radians(-90)
                linear = 0.0
            else:
                if min_index > 180:
                    angular = math.radians(min_index-360-90)
                else:
                    angular = math.radians(min_index-90)
        twist = Twist()
        twist.angular.z = angular
        twist.linear.x = linear
        self._cmd_pub.publish(twist)
        print(angular)


class WallFollower:

    def __init__(self):
        self.rate = rospy.Rate(20)
        self.state = 0
        self.botVelPub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laserSub = rospy.Subscriber('/scan', LaserScan, self.updateLaserData)
        self.laserData = {
            'r': 0,
            'fr': 0,
            'f': 0,
            'l': 0,
            'fl': 0
        }

    def updateLaserData(self, msg):
        self.laserData = {
            'fr': min(min(msg.ranges[30:60]), 10),
            'r': min(min(msg.ranges[61:120]), 10),
            'f': min(min(msg.ranges[330:359]), min(msg.ranges[0:30]), 10),
            'l': min(min(msg.ranges[260:299]), 10),
            'fl': min(min(msg.ranges[300:329]), 10)
        }
        print(self.laserData)

    def step(self):
        self.sensePlan()
        print("State %d" % self.state)
        twist = Twist()
        if self.state == 0:
            twist = self.findWall()
        elif self.state == 1:
            twist = self.turnLeft()
        elif self.state == 2:
            twist = self.followWall()
        elif self.state == 3:
            twist = self.turnRight()
            pass
        else:
            rospy.logerr('Unknown state!')

        self.botVelPub.publish(twist)

        self.rate.sleep()

    def sensePlan(self):
        r = self.laserData['r']
        fr = self.laserData['fr']
        f = self.laserData['f']
        l = self.laserData['l']
        fl = self.laserData['fl']

        d = 0.27
        state_description = ""

        if f < d:
            self.state = 1
            #if f > r < d:
            #    self.state = 2
            #else:
            #    self.state = 1
        #elif fr == 10 or fr < d:
        #    if r < d:
        #        self.state = 1
        #    else:
        #        self.state  = 2
        elif r < d:
            self.state = 2
        elif f > d and fr > d and r > d:
            self.state = 0
        else:
            state_description = 'unknown case'

        print(state_description)

    def findWall(self):
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0
        return twist

    def turnLeft(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0.2
        return twist

    def turnRight(self):
        twist = Twist()
        twist.angular.z = -0.2
        return twist

    def followWall(self):
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = -0.2
        return twist


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
        wall_follow_two = WallFollower()
        obstacle = Obstacle()
        while self.loc_feedback.cur_loc_state.ellipse_area > goal.epsilon:
            #wall_follow.step()
            #wall_follow_two.step()
            obstacle.wall_follow()
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
