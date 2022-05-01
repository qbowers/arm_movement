import rospy


import tty
import sys
import termios
import select

class Controller():
    def __init__(self):
        # old_settings = termios.tcgetattr(sys.stdin)
        # try:
        #     tty.setcbreak(sys.stdin.fileno())

        #     i = 0
        #     while 1:
        #         print(i)
        #         i += 1

        #         if self.isData():
        #             c = sys.stdin.read(1)
        #             if c == '\x1b':         # x1b is ESC
        #                 break

        # finally:
        #         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def isData():
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def loop_function(self, keystate):
        pass


if __name__ == '__main__':
    rospy.init_node("ArmController", anonymous=True)
    Controller()
    rospy.spin()
