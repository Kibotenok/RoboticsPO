# Лабораторная работа №3
# Реализация класса управления черепахой (вращение на месте и движение по прямой линии) 
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import radians, sqrt, pow


class  TurtleMove:

    def __init__(self, tolerance=0.01):
        """Move and rotate the turtle to target position
            :param tolerance: distance and angle tolerance
        """
        # Init node
        rospy.init_node('turtle1', anonymous=True)
        # Init publisher and subscriber to conrtol the turtle
        self.vel_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                             Twist,
                                             queue_size=10)
        self.pos_subscriber = rospy.Subscriber('/turtle1/pos',
                                                Pose, self.__update_pos)
        # Current turtle position
        self.cur_pos = Pose()
        
        self.dis_tolerance = tolerance
        self.angle_tolerance = tolerance
        # Dictionary with coefficients to choose moving/rotating direction
        # r - clockwise rotation; f - forward move;
        # l - counterclockwise rotation; b - backward move;
        self.way = {'l' or 'f': 1, 'r' or 'b': -1}

    def __update_pos(self, coord):
        """Callback method to update position coordinates
            :param coord: current position coordinates
        """ 
        self.cur_pos = coord

    def __remaining_distance(self, target):
        """Calculate remaining distance to target position
            :param target: target position coordinates

            :return remaining distance
        """
        return sqrt(pow(target.x-self.cur_pos.x, 2)+pow(target.y-self.cur_pos.y, 2))

    def __linear_vel(self, target):
        """Calculate linear velocity
            :param target: target position coordinates

            :return new linear velocity value
        """
        return 1.5 * self.__remaining_distance(target)

    def __rotate_vel(self, angle):
        """Calculate rotate velocity
            :param angle: angle in radians

            :return new rotate velocity value
        """
        return 6 * (angle - self.cur_pos.theta)

    def jumpt_to(self, dis):
        """Move the turtle in linear path without drawing
            :param dis: distance to move
        """
        pass
    
    def rotate(self, angle, way):
        """Rotate the turtle by specified angle
            :param angle: angle in degrees
            :param way: direction of rotation
        """
        vel = Twist()
        cur_angle = radians(angle)
        
        while cur_angle >= self.angle_tolerance:
            # Angular velocity
            vel.linear.x = 0
            vel.linear.y = 0
            vel.linear.z = 0
            
            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = self.way[way]*self.__rotate_vel(radians(angle))

            # Publish Twist object
            self.vel_publisher.publish(vel)
            self.rate.sleep()

            cur_angle = abs(radians(angle) - self.cur_pos.theta)

        # Stop the turtle
        vel.angular.z = 0
        self.vel_publisher.publish(vel)

        rospy.spin()

    def line_move(self, target, way):
        """Move the turtle in linear path
            :param target: target position coordinates
        """
        vel = Twist()
        
        while self.__remaining_distance(target) >= self.dis_tolerance:
            # Linear velocity
            vel.linear.x = self.way[way]*self.__linear_vel(target)
            vel.linear.y = 0
            vel.linear.z = 0

            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = 0

            # Publish Twist object 
            self.vel_publisher.publish(vel)
            self.rate.sleep()

        # Stop the turtle
        vel.linear.x = 0
        self.vel_publisher.publish(vel)

        rospy.spin()
