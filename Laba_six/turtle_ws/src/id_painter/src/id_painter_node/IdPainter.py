#!/usr/bin/env python3
import rospy
from turtlesim.srv import TeleportRelative
from std_srvs.srv import Empty
import os

from TurtleMove import TurtleMove

# Dictionary with commands to paint digits
NUMBERS = {'2': "frfrflflf",
           '3': "frfrfblfrfrr",
           '4': "lffblfrfr",
           '9': "flfflflflf",
           '0': "lflflfflflfbr"}


class IdPainter(TurtleMove):

    def __init__(self, tolerance=0.01):
        """Draw personal id number
            :param tolerance: distance and angle tolerance (default 0.01)
        """
        super().__init__(tolerance)
        # Dictionary with move methods
        self.action = {'f': [self.linear_move, 1], 'b': [self.linear_move, -1], 'r': [self.rotate, -1], 'l': [self.rotate, 1]}
        self.__settings()
    
    def __settings(self):
        """Set background color and pen options"""
        rospy.set_param("/turtlesim_node/background_g", 30)
        rospy.set_param("/turtlesim_node/background_b", 140)
        rospy.set_param("/turtlesim_node/background_r", 170)
        clear = rospy.ServiceProxy("clear", Empty)
        res = clear()
        # Set pen options
        self._set_pen()
        rospy.loginfo("Background and pen settings completed")
    
    def teleport(self, k, dis=0.7):
        """Relative teleport without drawing
            :param k: distance multiplier
            :param dis: line length (default 0.5)
        """
        self._set_pen(1)
        teleport = rospy.ServiceProxy("turtle1/teleport_relative", TeleportRelative)
        res = teleport(k*dis, 0)
        self._set_pen(0)
        
    def id_paint(self, com):
        """Draw personal id number
            :param com: string with commands to move the turtle
        """
        for act in com:
            self.action[act][0](self.action[act][1])


if __name__ == "__main__":

    try:
        tolerance = 0.00001
        painter = IdPainter(tolerance)
        rospy.loginfo("The turtle is ready. Tolerance for measurements is {:.5f}".format(tolerance))
        painter.start_pose(2.0, 6.0)
        
        for digit in "243904":
            painter.id_paint(NUMBERS[digit])
            rospy.loginfo("Number %s was painted by the turtle", digit)
            painter.teleport(2)
            rospy.loginfo("The turtle is ready to paint the next digit")

        rospy.loginfo("The turtle finished painting")
        painter.start_pose(6.0, 8.0)
        rospy.loginfo("The turtle is ready to paint something else")

        if input("Would you like to finish? [Y/n] ").lower().startswith('y'):
            rospy.loginfo("Start killing roscore process...")
            try:
                os.system("killall roslaunch")
            except OSError:
                rospy.logerr("OSError: Killing process failed")
            
    except rospy.ROSInterruptException:
        rospy.logerr("ROSError: Operation was interrupted")
