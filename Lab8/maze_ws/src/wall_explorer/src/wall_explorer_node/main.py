#!/usr/bin/env python3
import rospy
from MazeBot import MazeBot

if __name__ == "__main__":

    try:
        # Create the maze bot and start move
        maze_bot = MazeBot(0.01, 3, 0, 0)
        maze_bot.move()
    except rospy.ROSInterruptException:
        rospy.logerr("ROSError: Operation was interrupted")
        
