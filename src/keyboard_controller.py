#!/usr/bin/env python3

import rospy


# control imports
import keyboard


class KeyboardController():
    def __init__(self):
        rospy.Timer(rospy.Duration(1.0/20.0), self.get_keys)


    def get_keys(self, event):
        if keyboard.is_pressed("w"):
            print("Pressed W")
        

if __name__ == "__main__":
    rospy.init_node("keyboard_controller")
    brick_detector = KeyboardController()
    rospy.spin()