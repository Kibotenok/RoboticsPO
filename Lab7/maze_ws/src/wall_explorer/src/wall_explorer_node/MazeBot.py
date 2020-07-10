#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import random
from PID import PID


class MazeBot:

    def __init__(self, tolerance):
        """Control the movement of the robot in the maze
            :param tolerance: distance tolerance
        """
        self.tolerance = tolerance
        self._STEP = 17
        
        rospy.init_node("wall_explorer_node", anonymous=False) # Init node to control the turtle
        self._vel_publisher = rospy.Publisher('/cmd_vel', Twist,
                                                  queue_size=10)
        self._laser_subscriber = rospy.Subscriber('/base_scan', LaserScan,
                                                self._update_laser)
        self._laser = LaserScan()
        self._laser.ranges = [5.0 for i in range(90)]
        # Connection rate in Hz
        self._rate = rospy.Rate(10)

        self._dis_pid = PID(1, 0, 0)
        self._ang_pid = PID(60, 0, 0)
        rospy.loginfo("The bot was created successfully\nTolerance for distance is {:.3f}".format(self.tolerance))

    def _update_laser(self, data):
        """ Update laser data
            :param data: new data
        """
        self._laser = data

    def _potetional_field(self):
        """ Potentional field algorithm

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
        vectors = [np.min(self._laser.ranges[i:i+self._STEP]) for i in range(0, 73, 18)]
        dis = [all((vectors[2] >= 2.0, vectors[1] >= 1.0, vectors[3] >= 1.0)), vectors[0] < vectors[4], vectors[0] > vectors[4],
               vectors[1] > vectors[3]]
        dis.append(all((not dis[0], not dis[1], not dis[2], not dis[3])))
        count = 0
        index = [i for i in range(len(dis)) if dis[i] == 1]
        random.shuffle(index)
        choice = random.choice(index)
        if choice == 0:
            return vectors[2], 0.0
        elif choice == 1:
            return 0.5, vectors[0]/vectors[4]
        elif choice == 2:
            return 0.5, -vectors[4]/vectors[0]
        elif choice == 3:
            return vectors[2], -vectors[3]/vectors[1]
        else:
            return vectors[2], vectors[1]/vectors[3]
    
    def move(self):
        """Move the bot"""
        rospy.loginfo("The bot starts to move")
        vel = Twist()
        dis, ang = self._potetional_field()
        while dis >= self.tolerance:
            # Get distance and angular
            dis, ang = self._potetional_field()
            rospy.loginfo("New vector: distance - {0:.3f}; angle - {1:.3f}".format(dis, ang))
            # Calculate and publish new velocity values
            vel.angular.z = self._ang_pid.regulator(ang)
            vel.linear.x = self._dis_pid.regulator(dis)
            self._vel_publisher.publish(vel)
            self._rate.sleep()

        # Stop the bot
        vel.linear.x = 0
        vel.angular.z = 0
        self._vel_publisher.publish(vel)
        self._rate.sleep()
        rospy.loginfo("The bot was stopped")
        
