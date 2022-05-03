#!/usr/bin/env python2

import sys
import keyboard
import fcntl
import os
import termios
import tty
import copy
import math
import numpy as np

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

#import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
#from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class UR5Controller(object):
  """UR5Controller"""
  def __init__(self):
    super(UR5Controller, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    #self.command = outputMsg.Robotiq2FGripper_robot_output();

    self.home_pose = np.array([0.5, 0.0, 0.15, 0.0])
    #self.home_orientation_offsets = np.array([0.0, 90.0, 0.0]) 
    self.home_orientation_offsets = np.array([-180.0, 0.0, -180.0])

    self.assembly_origin_pose_offset = np.array([0.0, 0.0, 0.0, 0.0])
    self.assembly_origin_pose = np.array([0.6, -0.2, 0.15, 90.0])
    self.assembly_origin_pose = self.assembly_origin_pose + self.assembly_origin_pose_offset


    offset_case = 1

    if offset_case == 1:
      euler_angles = self.home_orientation_offsets * (pi/180.0)
      home_quaternion = quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2])

    self.teleop_speeds = np.array([0.005, 0.005, 0.005, 0.005])
    self.teleop_accelerations = np.array([0.05, 0.05, 0.05, 0.05])
    self.pose_acceleration = np.array([0.0, 0.0, 0.0, 0.0])
    self.pose_velocity = np.array([0.0, 0.0, 0.0, 0.0])
    self.max_pose_velocity = np.array([0.1, 0.1, 0.1, 0.1])
    self.pose_damping = np.array([0.7, 0.7, 0.7, 0.7])

  def plan_cartesian_path(self, scale=1, waypoints=[]):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL

  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL
  
  def rotate_end_effector(self, angle_change):
    # change the yaw angle of the end effector
    move_group = self.move_group

    joint_goal = move_group.get_current_joint_values()
    joint_goal[5] = joint_goal[5] + angle_change

    move_group.go(joint_goal, wait=True)
    move_group.stop()

  def get_pose_change_waypoint(self, pose_change):
    # pose_change [dx, dy, dz, dtheta]
    waypoints = []
    old_pose = self.move_group.get_current_pose().pose
    pose = copy.deepcopy(old_pose)
    pose.position.x += pose_change[0]
    pose.position.y += pose_change[1]
    pose.position.z += pose_change[2]

    if pose_change[3] != 0.0:
      _, _, gripper_angle = euler_from_quaternion(old_pose.orientation.x, old_pose.orientation.y, old_pose.orientation.z, old_pose.orientation.w)
      euler_angles = self.home_orientation_offsets * (pi/180.0) + np.array([0.0, 0.0, gripper_angle + pose_change[3]])
      quaternion = quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2])
      pose.orientation.x = quaternion[0]
      pose.orientation.y = quaternion[1]
      pose.orientation.z = quaternion[2]
      pose.orientation.w = quaternion[3]

    waypoints.append(pose)
    return waypoints

  def get_pose_waypoint(self, pose_array):
    # pose_array [dx, dy, dz, dtheta]
    waypoints = []
    old_pose = self.move_group.get_current_pose().pose
    poes = copy.deepcopy(old_pose)
    poes.position.x = pose_array[0]
    poes.position.y = pose_array[1]
    poes.position.z = pose_array[2]

    euler_angles = self.home_orientation_offsets * (pi/180.0) + np.array([0.0, 0.0, pose_array[3]]) * (pi/180.0)
      
    quaternion = quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2])
    poes.orientation.x = quaternion[0]
    poes.orientation.y = quaternion[1]
    poes.orientation.z = quaternion[2]
    poes.orientation.w = quaternion[3]
    waypoints.append(poes)
    return waypoints
 
def euler_from_quaternion(x, y, z, w):
  """
  Convert a quaternion into euler angles (roll, pitch, yaw)
  roll is rotation around x in radians (counterclockwise)
  pitch is rotation around y in radians (counterclockwise)
  yaw is rotation around z in radians (counterclockwise)
  """
  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  roll_x = math.atan2(t0, t1)

  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  pitch_y = math.asin(t2)

  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  yaw_z = math.atan2(t3, t4)

  return roll_x, pitch_y, yaw_z # in radians

def quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

# teleop control
def teleop(ur5):
  homing = False
  ur5.pose_velocity = np.array([0.0, 0.0, 0.0, 0.0]) 
  
  # use appropriate keyset is to move robot at faster or slower speeds
  # hold Shift to make move the robot at a slower speed
  slow_keyset = {"w", "s", "a", "d", "r", "f", "c", "v"}
  fast_keyset = {"i", "k", "j", "l", "y", "h", "b", "n"}

  try:
    c = sys.stdin.read(1)
    c = repr(c)[1:-1]
    print(c)

    if c.lower() == "q":
      return False

    speed_mult = 1 + 500 * (int(c.lower() == c)) + 1500 * int(c.lower() in fast_keyset)

    # translate the end effector in XYZ
    if c.lower() == "w" or c.lower() == "i":
      ur5.pose_velocity[0] = ur5.teleop_speeds[0] * speed_mult
    if c.lower() == "s" or c.lower() == "k":
      ur5.pose_velocity[0] = -ur5.teleop_speeds[0] * speed_mult
    if c.lower() == "a" or c.lower() == "j":
      ur5.pose_velocity[1] = ur5.teleop_speeds[1] * speed_mult
    if c.lower() == "d" or c.lower() == "l":
      ur5.pose_velocity[1] = -ur5.teleop_speeds[1] * speed_mult
    if c.lower() == "r" or c.lower() == "y":
      ur5.pose_velocity[2] = ur5.teleop_speeds[2] * speed_mult
    if c.lower() == "f" or c.lower() == "h":
      ur5.pose_velocity[2] = -ur5.teleop_speeds[2] * speed_mult

    # control the yaw angle of the end effector
    if c.lower() == "c" or c.lower() == "b":
      ur5.rotate_end_effector(ur5.teleop_speeds[3] * speed_mult)
    if c.lower() == "v" or c.lower() == "n":
      ur5.rotate_end_effector(-ur5.teleop_speeds[3] * speed_mult)

    # send end effector to the home position
    if c.lower() == " ":
      ur5.pose_acceleration = np.array([0.0, 0.0, 0.0, 0.0])
      ur5.pose_velocity = np.array([0.0, 0.0, 0.0, 0.0])
      waypoints = ur5.get_pose_waypoint(ur5.home_pose)
      homing = True

    if not homing:
      ur5.pose_velocity = np.clip(ur5.pose_velocity, -ur5.max_pose_velocity[0], ur5.max_pose_velocity[0])
      waypoints = ur5.get_pose_change_waypoint(ur5.pose_velocity)

    cartesian_plan, fraction = ur5.plan_cartesian_path(waypoints=waypoints)
    ur5.execute_plan(cartesian_plan)
  except IOError:
      pass 

  return True

def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the UR5 Interface Controller"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
    raw_input()
    ur5 = UR5Controller()
    print(ur5.move_group.get_current_pose().pose)

    rate = rospy.Rate(20) # 1 Hz

    run = True
    with raw(sys.stdin):
      with nonblocking(sys.stdin):
        while run:
          run = teleop(ur5)
          rate.sleep()

    print "============ Controller exiting!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

# teleop methods to get keyboard input
class raw(object):
    def __init__(self, stream):
        self.stream = stream
        self.fd = self.stream.fileno()
    def __enter__(self):
        self.original_stty = termios.tcgetattr(self.stream)
        tty.setcbreak(self.stream)
    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.stream, termios.TCSANOW, self.original_stty)

class nonblocking(object):
    def __init__(self, stream):
        self.stream = stream
        self.fd = self.stream.fileno()
    def __enter__(self):
        self.orig_fl = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.orig_fl | os.O_NONBLOCK)
    def __exit__(self, *args):
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.orig_fl)

def getch():
  fd = sys.stdin.fileno()
  old_settings = termios.tcgetattr(fd)
  try:
    tty.setraw(sys.stdin.fileno())
    ch = sys.stdin.readline(1)
 
  finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
  return ch

if __name__ == '__main__':
  main()
