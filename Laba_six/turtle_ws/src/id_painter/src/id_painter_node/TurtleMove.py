#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, TeleportAbsolute
from math import sqrt, pi, radians, sin, cos


class TurtleMove:

    def __init__(self, tolerance):
        """Control the turtle moving
            :param tolerance: distance and angle tolerance
        """
        self.tolerance = tolerance
        self.cur_pos = Pose()  # Turtle current position

        rospy.init_node("turtle_bot", anonymous=False) # Init node to control the turtle
        # Publisher and subscriber to communicate with the turtle
        self._pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose,
                                                self._update_pose)
        self._vel_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist,
                                              queue_size=10)
        # Connection rate in Hz
        self._rate = rospy.Rate(10)

    def _update_pose(self, coord):
        """Update current position value
            :param coord: new coordinates value
        """
        self.cur_pos = coord

    @classmethod
    def _set_pen(cls, switch=0):
        """Set pen options
            :param switch: turn pen on/off (default 0)
        """
        pen = rospy.ServiceProxy("turtle1/set_pen", SetPen)
        res = pen(255, 255, 255, 4, switch)

    def _to_rad(self, way, angle):
        """Convert angle value from degrees to radians
            :param way: move direction
            :param angle: angle value in degrees

            :return absolute angle value in radians
        """
        target = self.cur_pos.theta + way*radians(angle)
        # Border conditions
        if target > pi:
            return target - 2*pi
        elif target < -pi:
            return target + 2*pi
        else:
            return target

    def _to_coord(self, way, dis):
        """Convert distance value to coordinates
            :param way: move direction
            :param dis: line length

            :return absolute coordinates value
        """
        target = Pose()
        target.x = self.cur_pos.x + way*dis*cos(self.cur_pos.theta)
        target.y = self.cur_pos.y + way*dis*sin(self.cur_pos.theta)
        return target

    def _remaining_dis(self, target):
        """Calculate remaining distance to target
            :param target: target coordinates

            :return remaining distance
        """
        return sqrt(pow(target.x - self.cur_pos.x, 2) + pow(target.y - self.cur_pos.y, 2))

    def _remaining_angle(self, target):
        """Calculate remaining angle to target
            :param target: target angle

            :return remaining angle
        """
        multy = target*self.cur_pos.theta
        # Border conditions
        if (multy < 0.0) and (abs(multy) >= pi*pi/4):
            return 2*pi - (abs(self.cur_pos.theta) + abs(target))
        else:
            return abs(self.cur_pos.theta - target)
    
    def _linear_vel(self, target):
        """Calculate linear velocity
            :param target: target coordinates

            :return linear velocity value
        """
        return 1.5*self._remaining_dis(target)

    def _angular_vel(self, target):
        """Calculate angular velocity
            :param target: target angle

            :return angular velocity value
        """
        return 5*self._remaining_angle(target)
    
    def start_pose(self, x=0.0, y=0.0, theta=0.0):
        """Teleport the turtle to start position
            :param x: X coordinate (default 0.0)
            :param y: Y coordinate (default 0.0)
            :param theta: Absolute angle value in degrees(default 0.0)
        """
        self._set_pen(1)
        teleport = rospy.ServiceProxy("turtle1/teleport_absolute", TeleportAbsolute)
        res = teleport(x, y, radians(theta))
        self._set_pen(0)
        rospy.loginfo("Start position:\nX: {0}\nY: {1}\nTheta: {2}".format(x, y, theta))

    def rotate(self, way, angle=90):
        """Rotate the turtle by specified angle
            :param way: move direction
            :param angle: angle value in degrees (default 90)
        """
        vel = Twist()
        target = self._to_rad(way, angle)
        # Rotate the turtle
        while self._remaining_angle(target) >= self.tolerance:
            vel.angular.z = way*self._angular_vel(target)
            self._vel_publisher.publish(vel)
            self._rate.sleep()
        # Stop the turtle
        vel.angular.z = 0
        self._vel_publisher.publish(vel)
        self._rate.sleep()

    def linear_move(self, way, dis=1):
        """Move the turtle to specified distance
            :param way: move direction
            :param dis: line length (default 1)
        """
        vel = Twist()
        target = self._to_coord(way, dis)
        # Move the turtle
        while self._remaining_dis(target) >= self.tolerance:
            vel.linear.x = way*self._linear_vel(target)
            self._vel_publisher.publish(vel)
            self._rate.sleep()
        # Stop the turtle
        vel.linear.x = 0
        self._vel_publisher.publish(vel)
        self._rate.sleep()


