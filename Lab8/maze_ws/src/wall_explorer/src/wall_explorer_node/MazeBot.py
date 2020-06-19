#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class MazeBot:

    def __init__(self, tolerance):
        """Control the movement of the robot in the maze
            :param tolerance: distance tolerance
        """
        self.tolerance = tolerance
        self._STEP = 29
        
        rospy.init_node("wall_explorer_node", anonymous=False) # Init node to control the turtle
        self._vel_publisher = rospy.Publisher('/cmd_vel', Twist,
                                                  queue_size=10)
        self._laser_subscriber = rospy.Subscriber('/base_scan', LaserScan,
                                                self._update_laser)
        self._laser = LaserScan()
        self._laser.ranges = [1.0 for i in range(360)]
        self._laser.range_max = 10.0
        # Connection rate in Hz
        self._rate = rospy.Rate(10)
        rospy.loginfo("The bot was created successfully\nTolerance for distance is {:.3f}".format(self.tolerance))

    def _update_laser(self, data):
        """ Update laser data
            :param data: new data
        """
        self._laser = data

    def _random_goal(self):
        """ Choose random coordinates

            :return relative distance and angle
        """
        return abs(np.random.randn()*10), np.radians(np.random.randint(0, 360))

    def _toXY(self, dis, ang):
        """ Convert vector [dis, ang] to [x, y]
            :param dis: vector length
            :param ang: vector angle

            :return vector [x, y]
        """
            return [dis*np.cos(ang), dis*np.sin(ang)]
    
    def _potetional_field(self, goal):
        """ Potentional field algorithm
            :param goal: goal point

            :return length and angle of direction vector
        """
        """
        # Calculate direction vectors
        vectors = [[np.mean(self._laser.ranges[i:i+self._STEP]), np.radians(180)-self._laser.angle_increment*(i+self._STEP/2)] for i in range(0, 79, 6)]
        # Find max value
        goal = vectors[np.random.choice(np.argwhere(np.max(vectors, axis=0) == vectors).transpose()[0])]
        # Convert from [dis, ang] to [x, y]
        vectors = [[self._laser.range_max-x[0], x[1]+np.radians(180)] for x in vectors if x[0] <= self._laser.range_max and not(x[0] == goal[0])]
        goal = [goal[0]*np.cos(goal[1]), goal[0]*np.sin(goal[1])]
        vectors = np.asarray([[x[0]*np.cos(x[1]), x[0]*np.sin(x[1])] for x in vectors])
        np.append(vectors, goal)
        # Calculate final vector
        final_vector = np.sum(vectors, axis=0)
        rospy.loginfo(final_vector)
        rospy.loginfo(goal)
        angle = np.radians(90) - np.arctan(final_vector[1]/final_vector[0])
        length = np.linalg.norm(final_vector)/20
        return self._laser.ranges[45], -angle
        """
        vectors = [-500.0/x**2 if x > 0 else -500.0 for x in self._laser.ranges]
        for i in range(0, len(vectors)):
            vectors[i] = self._toXY(vectors[i], self._laser.angle_increment*i)
        vectors = np.asarray(vectors)
        np.append(vectors, goal)
        final_vector = np.sum(vectors, axis=0)
        return np.linalg.norm(final_vector), -np.arctan(final_vector[1]/final_vector[0])
        
    def _pid_control(self, target, kp):
        """ Velocity PID control
            :param target: target velocity
            :param kp: proportional coefficient
            
            :return velocity value
        """
        p_part = kp*target
        i_part = 0
        d_part = 0
        return p_part + i_part + d_part

    def move(self):
        """Move the bot"""
        rospy.loginfo("The bot starts to move")
        vel = Twist()
        clock = 3000
        goal = [0.0, 0.0]
        dis, ang = self._potetional_field(goal)
        while self._laser.ranges[90] >= self.tolerance:
            # Get distance and angular
            if clock >= 500:
                clock = 0
                goal = self._toXY(self._random_goal())
                
            dis, ang = self._potetional_field(goal)
            rospy.loginfo("New vector: distance - {0:.3f}; angle - {1:.3f}".format(dis, ang))
            # Calculate and publish new velocity values
            vel.angular.z = self._pid_control(ang, 70)
            vel.linear.x = self._pid_control(dis, 1.5)
            self._vel_publisher.publish(vel)
            self._rate.sleep()
            clock += 1

        # Stop the bot
        vel.linear.x = 0
        vel.angular.z = 0
        self._vel_publisher.publish(vel)
        self._rate.sleep()
        rospy.loginfo("The bot was stopped")
        
