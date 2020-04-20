# Лабораторная работа №3
# Алгоритм движения черепахи
#!/usr/bin/env python
from TurtleMove import TurtleMove
import rospy


# Dictionary with commands on how to draw a number
# f - forward move;
# b - backward move;
# r - clockwise rotation;
# l - counterclockwise rotation;
NUMBERS = {'2': 'frfrflfl',
           '3': 'frfrfblfrfrr',
           '4': 'lffblfrfr',
           '9': 'flfflflflf',
           '0': 'lflflfflflfbr'}


class IdPainter(TurtleMove):

    def __init__(self, dis, tolerance=0.01):
        """Print id by the turtle
            :param dis: line lenght
            :param tolerance: distance and angle tolerance
        """
        super(IdPainter, self).__init__(tolerance)
        self.dis = dis
        self.angle = 90
        self.moves = {'f' or 'b': self.line_move,
                      'r' or 'l': self.rotate}                     }
        self.__settings()

    @classmethod
    def __settings(clt):
        """Set parameters of window"""
        rospy.set_param('background_r', 0)
        rospy.set_param('backgroudn_b', 255)
        rospy.set_param('background_g', 255)

    def num_print(self, commands):
        """Move the turtle by using commands
            :param commands: string of commands
        """
        for command in commands:
            self.moves[command](,command) 

        self.jump_to(2*self.dis)


if __name__ == '__main__':

    print("Enter some settings")
    dis = input("Enter line lenght: ")
    tolerance = input("Enter distance tolerance: ")

    try:
        painter = IdPainter(dis, tolerance)

        for num in "243904":
            
            painter.num_print(NUMBERS[num])
        
    except rospy.ROSInterruptException:
        print("Error: ROSInterruptException")
    
