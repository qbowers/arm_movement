#!/usr/bin/python


import rospy
import numpy as np
import keyboard
from numpy.linalg import inv
import sys
import termios
import os
import tty
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import ColorRGBA, Float32, Bool, Int32
from user_input.msg import Velocity, JoyCmd
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math

class keyboard_obj(object):

	def __init__(self):
		rospy.init_node('keyboard', anonymous=True)
		self.joyconnPub= rospy.Publisher('/joy/connected', Bool, queue_size = 1)
		self.joycmdPub = rospy.Publisher('/joy/cmd', JoyCmd, queue_size = 10)
		self.dockPub = rospy.Publisher('/test/docking', Bool, queue_size = 1) # Our custom topic called /test/docking with message type Bool (boolean)
		# this message is set to true when a character is typed in the command line (see detectCmds_dock). Ex. you may want to set a message to true when you finish a task like docking or pick and place

		r = rospy.Rate(60)
		self.cmd_msg = JoyCmd()
		self.conn_msg = Bool()
		self.dock_msg = Bool()
		self.conn_msg.data = True
		self.dock_msg.data = False
		self.mult = 1.0/3.0
		while not rospy.is_shutdown():
			self.cmd_msg = self.detectCmds()
			self.dock_msg = self.detectCmds_dock()
			# Publish at a frequency of 60 Hz
			self.joyconnPub.publish(self.conn_msg)
			self.joycmdPub.publish(self.cmd_msg)
			self.dockPub.publish(self.dock_msg)

			r.sleep()

	def getch(self):
		fd = sys.stdin.fileno()
		old_settings = termios.tcgetattr(fd)
		try:
			tty.setraw(sys.stdin.fileno())
			ch = sys.stdin.readline(1)
	 
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
		return ch

	def detectCmds_dock(self):
		char = self.getch()
		print(char)
		msg = Bool()

		if 'p' in char:
			msg.data = True
		elif 'o' in char:
			msg.data = False
		return msg

	def detectCmds(self):
		char = self.getch()
		print(char)
		msg = JoyCmd()
		#Detect forward and backward keys assigned to 'w' and 's'
		if 'w' in char:
			msg.axis1 =  1.0
		elif 's' in char:
			msg.axis1 = msg.axis1 - 1.0

		#Detect left and right keys 'a' and 'd'
		elif 'a' in char:
			msg.axis2 =  1.0
		elif 'd' in char:
			msg.axis2 = msg.axis2 - 1.0

		#Detect rotation keys 'q' for counterclockwise, 'e' for clockwise

		elif 'q' in char:
			msg.axis3 =  1.0
		elif 'e' in char:
			msg.axis3 = msg.axis3 - 1.0

		#Detect keys for multipliers
		elif 'h' in char:
			self.mult = 1.0/3.0
		elif 'j' in char:
			self.mult = 2.0/3.0
		elif 'k' in char:
			self.mult = 1.0

		#Dummy character that also works to stop the vehicle
		elif '0' in char:
			pass

		msg.axis1 = msg.axis1*self.mult
		msg.axis2 = msg.axis2*self.mult
		msg.axis3 = msg.axis3*self.mult
		return msg



if __name__=='__main__':
	node = keyboard_obj()
	


# var joy_connected_topic = new ROSLIB.Topic({
#     ros: ros,
#     name: '/joy/connected',
#     messageType: 'std_msgs/Bool'
# })
# var joy_cmd_topic = new ROSLIB.Topic({
#     ros: ros,
#     name: '/joy/cmd',
#     messageType: 'user_input/JoyCmd'
# })