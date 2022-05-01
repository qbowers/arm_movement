import rospy
import numpy as np


# control imports
import sys
import pygame
import time

# moveit imports
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class Controller():
    def __init__(self):


        # init keyboard inputs
        pygame.init()
        rospy.sleep(1)
        display = pygame.display.set_mode((300,300))

        # init moveit
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        # group_name = "panda_arm"
        group_name = "manipulator" # or possible "endeffector". Unsure.
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)


        target_pose = geometry_msgs.msg.Pose()
        target_pose.orientation.w = 1
        target_pose.position.x = 0
        target_pose.position.y = 0
        target_pose.position.z = 0

        rospy.logwarn("Controller Online")


        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            keystate = self.get_keystate()
            self.update_target(target_pose, keystate)
            self.loop_function( target_pose )
            rate.sleep()


    def get_keystate(self):
        events=pygame.event.get()

        # Close cleanly
        for event in events:
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
        
        # NOTE: Dont use numpy in this loop. pygame freaks out
        pressed = pygame.key.get_pressed()
        
        keys = [pygame.K_w, pygame.K_a, pygame.K_s, pygame.K_d, pygame.K_q, pygame.K_e, pygame.K_r, pygame.K_f]

        values = [False] * len(keys)
        for i in range(len(keys)):
            values[i] = pressed[keys[i]]

        return values
            
    def update_target(self, target_pose, keystate):
        # w & s keys
        if keystate[0]:
            target_pose.position.y += 0.05
        elif keystate[2]:
            target_pose.position.y -= 0.05
        
        if keystate[1]:
            target_pose.position.x += 0.05
        elif keystate[3]:
            target_pose.position.x -= 0.05
        
        if keystate[6]:
            target_pose.position.z += 0.05
        elif keystate[7]:
            target_pose.position.z -= 0.05

    # def isData():
    #     return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def loop_function(self, target_pose):

        # pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.orientation.w = 1.0
        # pose_goal.position.x = 0.4
        # pose_goal.position.y = 0.1
        # pose_goal.position.z = 0.4

        self.move_group.set_pose_target(target_pose)
        plan = self.move_group.go(wait=False)
        


if __name__ == '__main__':
    rospy.init_node("ArmController", anonymous=True)
    Controller()
    rospy.spin()
