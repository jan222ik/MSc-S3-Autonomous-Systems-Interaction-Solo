#! /usr/bin/env python

import rospy
import actionlib
import math
import tf
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from transformations_odom.msg import PoseTF, PoseInMap
# from traverse_path.msg import TraversePathAction, TraversePathActionResult, TraversePathActionFeedback, TraversePathResult
from geometry_msgs.msg import Twist, Pose
from plodding.msg import PlodAction, PlodActionResult, PlodResult, PlodActionFeedback
from turtlebot3_msgs.msg import Sound


class PloddingTurtle:
    def __init__(self):
        rospy.init_node("plodding", log_level=rospy.INFO, anonymous=True)
        self.rate = rospy.Rate(10)
        self.goalDistanceThreshold = rospy.get_param("/plodding_goalDistanceThreshold", 0.03)
        self.goalAngleThreshold = rospy.get_param("/plodding_goalAngleThreshold", 0.2)
        self.goalPoseAngleThreshold = rospy.get_param("/plodding_goalPoseAngleThreshold", 0.9)
        self.isGainingDistanceThreshold = rospy.get_param("/plodding_isGainingDistanceThreshold", 0.2)
        self.LINEAR_MAX_SPEED = rospy.get_param("/plodding_linear_max_speed", 0.3)
        self.lastDistance = 10000000
        self.state = State_Idle()
        self.logGoalCount = 0
        self.logSteer = 0.0
        self.logASpeed = 0.0
        self.logLSpeed = 0.0
        self.logDistance = 0.0
        self.logLastDistance = 0.0
        self.logBranch = 0
        self.isDone = False
        self.waitAfter = False
        rospy.loginfo("Plodding: Startup")
        self.pubTwist = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pubSound = rospy.Publisher("sound", Sound, queue_size=10)
        self.currentPose = Pose()
        self.mapPose = PoseInMap()
        self.goalPose = Pose()
        self.subRobotPose = rospy.Subscriber('pose_tf', PoseTF, self._subNextPose)
        rospy.logdebug("Plodding: Starting Action Server")
        self.actionServer = actionlib.SimpleActionServer("plodding_action_server", PlodAction,
                                                         execute_cb=self._nextActionPose, auto_start=False)
        self.actionServer.start()
        rospy.logdebug("Plodding: Started Action Server")

    def _subNextPose(self, data):
        self.mapPose = data.mapPose
        self.currentPose = data.originalPose

    def _nextActionPose(self, data):
        rospy.logdebug("Plodding: Next Action Pose {}".format(data))
        self.logGoalCount += 1
        self.externalCancel = False
        self.isDone = False
        self.goalPose = data.target
        self.waitAfter = not data.waitAfter
        self.lastDistance = 10000000

        self._changeState(State_Plodding())

        result = PlodActionResult()
        result.result = PlodResult()
        result.result.plod_has_finished = Bool()
        feedback = PlodActionFeedback()

        while not rospy.is_shutdown() and not self.externalCancel and not self.isDone:
            self.goalDistanceThreshold = rospy.get_param("/plodding_goalDistanceThreshold", 0.03)
            self.goalAngleThreshold = rospy.get_param("/plodding_goalAngleThreshold", 0.9)
            self.isGainingDistanceThreshold = rospy.get_param("/plodding_isGainingDistanceThreshold", 0.2)
            self.LINEAR_MAX_SPEED = rospy.get_param("/plodding_linear_max_speed", 0.3)
            self.goalPoseAngleThreshold = rospy.get_param("/plodding_goalPoseAngleThreshold", 0.9)
            if self.actionServer.is_preempt_requested():
                rospy.loginfo("Plodding: Requested Cancel")
                self.stop()
                self._changeState(State_Idle())
                self.externalCancel = True
            else:
                self._changeState(nextState=self.state.run(self))

            feedback.feedback.state = String(data=self.state.name())
            self.actionServer.publish_feedback(feedback.feedback)

        result.result.plod_has_finished.data = self.isDone
        self.actionServer.set_succeeded(result.result)

    def _changeState(self, nextState):
        if nextState != self.state:
            rospy.loginfo("Plodding: Transition {} State to {} State".format(self.state.name(), nextState.name()))
            self.state = nextState

    def playSound(self):
        rospy.loginfo("Plodding: Play Tag Sound")
        self.pubSound.publish(Sound(value=3))

    @staticmethod
    def calcDistance(first, second, roundingPos=4):
        """
            Calculate euclidean distance between two poses.
        """
        return math.sqrt(
            pow((first.position.x - round(second.position.x, roundingPos)), 2)
            + pow((first.position.y - round(second.position.y, roundingPos)), 2)
        )

    def calcAngle(self, first, second, roundingPos=4):
        y = first.position.y - round(second.position.y, roundingPos)
        x = first.position.x - round(second.position.x, roundingPos)
        angle = math.atan2(y, x)
        return angle

    def calcAngleTowardsPose(self, goal):
        orientation_q = self.currentPose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)

        touchX = goal.x
        touchY= goal.y
        centerX = self.currentPose.position.x
        centerY = self.currentPose.position.y
        deltaX = centerX - touchX
        deltaY = centerY - touchY
        tween =  math.atan2(deltaY, deltaX)

        angDist = shortest_angular_distance(yaw, tween)

        return angDist

    def angularSpeedFor(self, currentAngle, targetAngle):
        speed = 0.18
        radCurrent = self.assurePositiveAndIn2Pi(currentAngle)
        radTarget = self.assurePositiveAndIn2Pi(targetAngle)
        diff = (radCurrent - radTarget) % (2 * math.pi)
        # self.logSteer = diff
        if not diff < math.pi:
            speed *= -1

        return speed


    def angularSpeedFor2(self, angle):
        speed = 0.18
        self.logSteer = angle
        if not angle < 0:
            speed *= -1

        return speed

    @staticmethod
    def assurePositiveAndIn2Pi(rad):
        return (rad + 2 * math.pi) % (2 * math.pi)

    def publishTwistMessage(self, linearSpeed, angularSpeed):
        twist = Twist()
        twist.linear.x = linearSpeed
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = angularSpeed
        self.pubTwist.publish(twist)
        self.logASpeed = angularSpeed
        self.logLSpeed = linearSpeed
        self.log()
        self.rate.sleep()

    def log(self):
        rospy.loginfo(
            "[{}, {}, {}] Speeds: linear:{:2.2f} angular:{:2.2f} AngDiff:{:2.8f} Distance:{:3.8f} LastDistance:{:3.8f}"
                .format(
                self.logGoalCount,
                self.state.name(),
                self.logBranch,
                self.logLSpeed,
                self.logASpeed,
                self.logSteer,
                self.logDistance,
                self.logLastDistance
            )
        )

    def stop(self):
        self.publishTwistMessage(0, 0)

    def isInGoalDistanceRange(self):
        distance = self.calcDistance(first=self.goalPose, second=self.currentPose)
        self.logLastDistance = self.lastDistance
        self.logDistance = distance
        isGainingDistance = abs(self.lastDistance - distance) <= self.isGainingDistanceThreshold
        atGoal = distance <= self.goalDistanceThreshold
        if distance < self.lastDistance:
            self.lastDistance = distance
        return atGoal, isGainingDistance

    def isInGoalRotationRange(self):
        angleRange = abs(self.calcAngle(first=self.goalPose, second=self.currentPose) - self.mapPose.angle)
        self.logSteer = angleRange
        return angleRange <= self.goalAngleThreshold

    def isInGoalPoseRotationRange(self):
        angleRange = abs(abs(self.calcAngleTowardsPose(self.goalPose.position)) - math.pi)
        self.logSteer = angleRange
        return angleRange <= self.goalPoseAngleThreshold


class StateInterface:
    def name(self):
        pass

    def run(self, turtle):
        pass


class State_Idle(StateInterface):
    def name(self): return "Idle"

    def run(self, turtle):
        return self


class State_Plodding(StateInterface):
    def name(self):
        return "Plodding"

    def run(self, turtle):
        isInGoalDistanceRange, isGainingDistance = turtle.isInGoalDistanceRange()
        isInGoalPoseRotationRange = turtle.isInGoalPoseRotationRange()
        if isInGoalDistanceRange and isInGoalPoseRotationRange:
            # Exit Goal Reached
            turtle.stop()
            if turtle.waitAfter:
                return State_Timeout()
            else:
                turtle.isDone = True
                return State_Idle()

        linear = 0
        angular = 0

        turtle.logBranch = 0
        if not isInGoalPoseRotationRange:
            turtle.logBranch = 1
            # Location Reached but Rotation is still needed
            angular = turtle.angularSpeedFor2(
                turtle.calcAngleTowardsPose(turtle.goalPose.position)
            )
            """
            turtle.angularSpeedFor(
                currentAngle=turtle.calcAngle(
                    first=turtle.goalPose,
                    second=turtle.currentPose
                ),
                targetAngle=turtle.mapPose.angle
            )"""
        elif not isInGoalDistanceRange and not isGainingDistance:
            turtle.logBranch = 2
            linear = turtle.LINEAR_MAX_SPEED
        else:
            turtle.logBranch = 3
            angular = turtle.angularSpeedFor(
                currentAngle=turtle.calcAngle(
                    first=turtle.goalPose,
                    second=turtle.currentPose
                ),
                targetAngle=turtle.mapPose.angle
            )
            linear = turtle.LINEAR_MAX_SPEED / 2

        turtle.publishTwistMessage(
            linearSpeed=linear,
            angularSpeed=angular
        )

        return self


class State_Timeout(StateInterface):

    def __init__(self):
        self.time = None
        self.timeoutConstant = 2

    def name(self):
        return "Timeout"

    def run(self, turtle):
        if self.time is None:  # First Run: Start Timer
            self.time = rospy.get_rostime().secs + self.timeoutConstant
            turtle.playSound()

        if self.time <= rospy.get_rostime().secs:
            self.time = None
            turtle.isDone = True
            return State_Idle()

        return self


class State_GoalRotation(StateInterface):

    def __init__(self):
        pass

    def name(self):
        return "GoalRotation"

    def run(self, turtle):
        pass


class State_AvoidCollision:
    def name(self): return "Avoid Collision"

    def run(self, turtle):
        return self


def main():
    try:
        node = PloddingTurtle()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# Not our code:
from math import pi


def normalize_angle_positive(angle):
    """ Normalizes the angle to be 0 to 2*pi
        It takes and returns radians. """
    return angle % (2.0 * pi)


def normalize_angle(angle):
    """ Normalizes the angle to be -pi to +pi
        It takes and returns radians."""
    a = normalize_angle_positive(angle)
    if a > pi:
        a -= 2.0 * pi
    return a


def shortest_angular_distance(from_angle, to_angle):
    """ Given 2 angles, this returns the shortest angular
        difference.  The inputs and ouputs are of course radians.

        The result would always be -pi <= result <= pi. Adding the result
        to "from" will always get you an equivelent angle to "to".
    """
    return normalize_angle(to_angle - from_angle)

if __name__ == '__main__':
    main()
