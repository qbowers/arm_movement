import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import rospy
import numpy as np

class Controller():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        # group_name = "panda_arm"
        group_name = "manipulator" # or possible endeffector. Unsure.
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)


        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()
        rospy.logwarn("Controller Online")



        # joint_goal = move_group.get_current_joint_values()
        # joint_goal[0] = 0
        # joint_goal[1] = -pi/4
        # joint_goal[2] = 0
        # joint_goal[3] = -pi/2
        # joint_goal[4] = 0
        # joint_goal[5] = pi/3

        # move_group.go(joint_goal, wait=True)
        # move_group.stop()

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        move_group.set_pose_target(pose_goal)

        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()



if __name__ == '__main__':
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    Controller()
    rospy.spin()