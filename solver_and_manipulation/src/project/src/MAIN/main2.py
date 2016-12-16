import sys,argparse,socket,os, thread
from threading import Thread

import tf, moveit_commander
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import copy

from moveit_msgs.msg import *
from geometry_msgs.msg import PoseStamped, Pose

import constant_parameters
from constant_parameters import left_hang, right_hang, cube_table, camera_threads
from constant_parameters import Joint_Names
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String, Int16MultiArray

import numpy as np
from numpy import pi
import math
from sensor_msgs.msg import Image # ROS Image message, Warning!! This is different from OpenCV Image message.
import cv_bridge
import exp_quat_func as eqf
from baxter_core_msgs.srv import (
    ListCameras
)

class Magic_Cube: 

	Solutions = []

	def __init__(self):
		print('\nStart Initialization!!!!\n')
		#initialize a node called arm
		rospy.init_node('Master', anonymous=True)		

		##### Initialize Camera Information Subscriber #####
		# rospy.init_node('Camera_Iformation_Subscriber', anonymous=True)

		###### Initialize moveit_commander #####
		moveit_commander.roscpp_initialize(sys.argv)

		#set up MoveIt
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()

		self.MoveIt_left_arm = moveit_commander.MoveGroupCommander('left_arm')
		self.MoveIt_left_arm.set_planner_id('RRTConnectkConfigDefault')
		self.MoveIt_left_arm.set_planning_time(10)
		self.MoveIt_left_arm.set_goal_position_tolerance(0.01)
		self.MoveIt_left_arm.set_goal_orientation_tolerance(0.01) 

		self.MoveIt_right_arm = moveit_commander.MoveGroupCommander('right_arm')
		self.MoveIt_right_arm.set_planner_id('RRTConnectkConfigDefault')
		self.MoveIt_right_arm.set_planning_time(10)
		self.MoveIt_right_arm.set_goal_position_tolerance(0.01)
		self.MoveIt_right_arm.set_goal_orientation_tolerance(0.01) 

		self.Left_Arm = baxter_interface.Limb('left')
		self.Right_Arm = baxter_interface.Limb('right')

		self.Left_Gripper = baxter_interface.Gripper('left', CHECK_VERSION)
		self.Left_Gripper.set_holding_force(80) 
		# print('Calibrating left gripper...')
		# self.Left_Gripper.calibrate()
		# rospy.sleep(0.5)

		self.Right_Gripper = baxter_interface.Gripper('right', CHECK_VERSION)
		self.Right_Gripper.set_holding_force(80) 
		# print('Calibrating right gripper...')
		# self.Right_Gripper.calibrate()
		# rospy.sleep(0.5)
		#relative transformation 
		self.TFlistener = tf.TransformListener()

		self.HoldHand = 'right'
		self.RotHand = 'left'
		self.LeftMarker = None
		self.RightMarker = None
	
		print("Getting robot state... ")
		self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
		self._init_state = self._rs.state().enabled
		print("Enabling robot... ")
		self._rs.enable()		

	def Camera_Control(self, camera_name, command):
		assert (command =='open' or command=='close'), 'only two types of command are allowed. \n'
		assert camera_name in camera_threads, 'Not given a valid camera name! \n'
		def Parallel_Thread(string):
			os.system(string)

		ls = rospy.ServiceProxy('cameras/list', ListCameras)
		rospy.wait_for_service('cameras/list', timeout=10)
		resp = ls()
		commands = {'left_hand_camera':"Thread(target=Parallel_Thread, args=('cd ../launch; roslaunch left_wrist.launch',))",
		            'right_hand_camera': "Thread(target=Parallel_Thread, args=('cd ../launch; roslaunch right_wrist.launch',))",
		            'head_camera': "Thread(target=Parallel_Thread, args=('cd ../launch; roslaunch head_cam.launch',))"}
		if camera_name not in resp.cameras:
			print('Cannot control ' + camera_name + ', other 2 cameras are open. ')
		else:
			if command == 'open':
				if not camera_threads[camera_name]:
					camera_threads[camera_name] = eval(commands[camera_name])
					camera_threads[camera_name].start()
					print(camera_name +' is opened.')
				else:
					print(camera_name + ' has already been launched!')
			elif command == 'close':
				print('I got your command!\n')		
				if camera_name == 'left_hand_camera':
					camera = baxter_interface.CameraController('left_hand_camera')
				elif camera_name == 'right_hand_camera':
					camera = baxter_interface.CameraController('right_hand_camera')
				else:
					camera = baxter_interface.CameraController('head_camera')
				if  camera_threads[camera_name]:
					print('Closing!!!!')
					camera_threads[camera_name] = None
				camera.close()
				print(camera._id+' is closed.')
				
	# The following three functions describe how to control the Gripper 
	def Gripper_Control(self, Gripper, command):
		assert (command == 'open' or command == 'close'), 'The Gripper can only be open or close?'
		if command == 'open':
			Gripper.open() 
		elif command == 'close':
			Gripper.close()
		else:
			print('The gripper command is not valid')

	def Trans_Angle(self, angle):
		if angle > pi:
			angle = angle - 2 * pi
		elif angle < -pi:
			angle += 2 * pi
		return angle

	def Initial_Check(self):
		def Joint_Convert(n, Handness):
			assert n>=0 , 'The extracted joint state index must larger or equal zero.'
			joint_angles = constant_parameters.joint_states[n]
			if Handness == 'left':
				joint_angles = {'left_s0':joint_angles[4], 'left_s1':joint_angles[5], 'left_e0':joint_angles[2], 'left_e1':joint_angles[3],
								'left_w0':joint_angles[6], 'left_w1':joint_angles[7], 'left_w2':joint_angles[8]}
			elif Handness == 'right':
	        	#need the correct joint angles input
				joint_angles = {'right_s0':joint_angles[11], 'right_s1':joint_angles[12], 'right_e0':joint_angles[9], 'right_e1':joint_angles[10], 
								'right_w0':joint_angles[13], 'right_w1':joint_angles[14], 'right_w2':joint_angles[15]}
			else:
				print('Invalid input Handness...')
			return joint_angles

		print('Initial Check for six sides.')
		# initial state
		left_angles = Joint_Convert(0,'left')
		right_angles = Joint_Convert(0,'right')
		self.Move_Joints('left',left_angles)
		self.Move_Joints('right',right_angles)
		self.Gripper_Control(self.Left_Gripper, 'open')
		self.Gripper_Control(self.Right_Gripper, 'open')
		rospy.sleep(0.5)

		raw_input('\n Press enter to continue!')
		
		#prone to get cube
		left_angles = Joint_Convert(1,'left')
		self.Move_Joints('left',left_angles)
		rospy.sleep(0.5)
		#There is a thing that can be optimaized
		raw_input('\n Please place the cube and press the enter key to start scan cube.')

		self.Gripper_Control(self.Left_Gripper, 'close')
		rospy.sleep(1)
		
		#Check 1st side
		print('Checking 1st side ...')
		left_angles = Joint_Convert(2,'left')
		self.Move_Joints('left',left_angles)
		print('Finish checking 1st side')
		rospy.sleep(0.5)
		self.Send_Images()
		rospy.sleep(0.5)

		#Check 2nd side
		raw_input('\n Press enter to continue!')
		print('Checking 2nd side')
		self.Rotation('left',radian=1.58)
		raw_input('Finish checking 2nd side')
		rospy.sleep(0.5)
		self.Send_Images()
		rospy.sleep(0.5)

		# Check 3rd side
		raw_input('Checking 3rd side ...')
		left_angles = Joint_Convert(3,'left')
		self.Move_Joints('left',left_angles)
		rospy.sleep(0.5)
		self.Send_Images()
		rospy.sleep(0.5)
		print('Finish checking 3rd side')

		raw_input('Checking 4th side ...')
		# print('Checking 4th side')
		left_angles = Joint_Convert(4,'left')
		self.Move_Joints('left',left_angles)
		right_angles = Joint_Convert(4,'right')
		self.Move_Joints('right',right_angles)
		raw_input('Next step.....')
		self.RightMarker = self.Get_Marker('right')

		_, orientation = self.TFlistener.lookupTransform('/base', 'right_gripper', rospy.Time(0))
		start_position = self.Get_End_Point_Positon('right')					
		end_position = [0.614, -0.008, 0.438]
		self.IK_MoveIt(self.MoveIt_right_arm, rot=orientation,StartPosition=start_position,EndPosition=end_position)
		rospy.sleep(0.5)

		self.Gripper_Control(self.Right_Gripper,'close')
		rospy.sleep(1)
		self.Gripper_Control(self.Left_Gripper,'open')

		_, orientation = self.TFlistener.lookupTransform('/base', 'left_gripper', rospy.Time(0))
		start_position = self.Get_End_Point_Positon('left')	
		middle_position = [start_position[0], start_position[1], start_position[2]+0.5]				
		end_position = [0.617, 0.220, 0.457]
		# right_angles = Joint_Convert(7,'left')
		# self.Move_Joints('left',left_angles)
		raw_input("2222222222222222222222222222")
		self.IK_MoveIt(self.MoveIt_left_arm, rot=orientation,StartPosition=start_position,MiddlePosition=middle_position, EndPosition=end_position)
		rospy.sleep(1)

		right_angles = Joint_Convert(5,'right')
		rospy.sleep(2)
		self.Move_Joints('right',right_angles)
		print('Finish checking 4th side')
		rospy.sleep(0.5)
		self.Send_Images()
		rospy.sleep(0.5)

		raw_input('Checking 5th side')
		self.Rotation('right',radian=3.14)
		print('Finish checking 5th side')
		rospy.sleep(0.5)
		self.Send_Images()
		rospy.sleep(0.5)
		
		raw_input('Checking 6th side')
		right_angles = Joint_Convert(6,'right')
		self.Move_Joints('right',right_angles)
		print('Finish checking last side')
		rospy.sleep(0.5)
		self.Send_Images()
		rospy.sleep(0.5)

		raw_input('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
		self.Camera_Control('head_camera','close')
		self.Camera_Control('right_hand_camera','open')
		self.Camera_Control('left_hand_camera','open')
		raw_input('Finish Initial Check!!!!')

	def Send_Images(self):
		def callback(msg):
			pass
		pub = rospy.Publisher('cube_images', Image, queue_size=10)
		rospy.Subscriber('/cameras/head_camera/image', Image, callback)
		while True:
			image_msg = rospy.wait_for_message('/cameras/head_camera/image',Image)
			if image_msg:
				pub.publish(image_msg)
				break

	# Move to some place by specifying joint angles
	def Move_Joints(self, Handness, joint_states):
		assert type(joint_states) == dict, 'Joint_States should be dictionary!\n Here are Joint Names: ' + Joint_Names
		# angles = dict(zip(self.joint_names(),[0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]))
		if Handness =='left':
			self.Left_Arm.move_to_joint_positions(joint_states, timeout = 15)
		elif Handness == 'right':
			self.Right_Arm.move_to_joint_positions(joint_states, timeout = 15)
		else:
			print('Invalid input parameter for Move_Joints function.')

	def Get_Marker(self, Handness):
		assert Handness=='left' or Handness=='right', 'The Handeness is only "left" or "right"'
		def callback(msg):
			pass
		detected = {}
		topic = '/ar_pose_marker'
		rospy.Subscriber(topic, AlvarMarkers, callback)
		while True:
			msg1 = rospy.wait_for_message(topic, AlvarMarkers)
			for marker in msg1.markers:
				detected = marker.id
				if detected and Handness in marker.header.frame_id: 
					return 'ar_marker_'+str(detected)  
				
	def IK_MoveIt(self,Arm,rot,StartPosition=False, MiddlePosition=False,EndPosition=False , Accuracy=0.03):

		waypoints = []  
		wpose = Pose()
		wpose.orientation.x = rot[0]
		wpose.orientation.y = rot[1]
		wpose.orientation.z = rot[2]
		wpose.orientation.w = rot[3]

		# first point
		wpose.position.x = StartPosition[0]
		wpose.position.y = StartPosition[1]
		wpose.position.z = StartPosition[2]
		waypoints.append(copy.deepcopy(wpose))

		# Middle Point (if existed)
		if MiddlePosition != False :
			Number = len(MiddlePosition)/3 - 1
			for i in range(Number):
				wpose.position.x = MiddlePosition[i*3]
				wpose.position.y = MiddlePosition[i*3+1]
				wpose.position.z = MiddlePosition[i*3+2]
				waypoints.append(copy.deepcopy(wpose))
		# End point
		wpose.position.x = EndPosition[0]
		wpose.position.y = EndPosition[1]
		wpose.position.z = EndPosition[2]
		waypoints.append(copy.deepcopy(wpose))
		# print(waypoints)

		(plan, fraction) = Arm.compute_cartesian_path(
		                           waypoints,   # waypoints to follow
		                           0.01,        # eef_step
		                           0.0)         # jump_threshold   
		Arm.execute(plan) 
		print('This is my end position: ',EndPosition)

	def Get_End_Point_Positon(self, Handness):
		if Handness == 'left':
			arm = self.Left_Arm
		else:
			arm = self.Right_Arm
		for i in range(50):
			POSE = arm.endpoint_pose()
			EndPoint = [POSE['position'].x ,POSE['position'].y, POSE['position'].z]
		return EndPoint

	def Grab_Cube(self,Handness,AR_marker,Accuracy=0.001,Marker_z=0,Base_Offsets=[0,0,0]):
		#There are two direction that can grasp the Arm, one is vertical and another is horizontal.
	
		if Handness =='left':
			move_arm = self.MoveIt_left_arm
			arm = self.Left_Arm
			gripper = self.Left_Gripper
			self.LeftMarker = cube_table[AR_marker]
		elif Handness == 'right':
			move_arm = self.MoveIt_right_arm
			arm = self.Right_Arm
			gripper = self.Right_Gripper
			self.RightMarker = cube_table[AR_marker]
		else:
			print('Invalid input parameter for Grab_Cube function.')
		
		self.Gripper_Control(gripper,'open')
		_, orientation = self.TFlistener.lookupTransform('/base', Handness + '_gripper', rospy.Time(0))
		marker_position, rot= self.TFlistener.lookupTransform('/base', AR_marker,rospy.Time(0))
		start_position = self.Get_End_Point_Positon(Handness)							
		omega, theta = eqf.quaternion_to_exp(rot)
		R1 = eqf.create_rbt(omega, theta, marker_position)

		while True:
			Marker_z = raw_input('Please input marker offset z: \n')
			Marker_z = float(Marker_z)
			Base_Offsets = raw_input('Please input Base offsets x, y, z: \n')
			Base_Offsets = map(float, Base_Offsets.split())
			if Marker_z == 999:
				break
			R2 = np.matrix([[1,0,0,0],
							[0,1,0,0],
							[0,0,1,Marker_z],
							[0,0,0,1]])
			R = np.matrix(R1) * R2
			end_position = [R[0,3] + Base_Offsets[0] ,R[1,3] + Base_Offsets[1],R[2,3] + Base_Offsets[2]]
			## MoveIt
			raw_input('Start IK_MoveIt!!!')
			print('start position:', start_position)
			print('end position: ', end_position)
			self.IK_MoveIt(move_arm,rot=orientation, StartPosition=start_position,EndPosition=end_position, Accuracy=Accuracy) #MiddlePosition=middle_position,
		
		raw_input('End Start IK_MoveIt!!! Closing ....')
		rospy.sleep(0.5)
		self.Gripper_Control(gripper, 'close')

	def Leave_Cube(self, Handness, Offsets=[0,0,0]):
		if Handness =='left':
			self.Gripper_Control(self.Left_Gripper,'open')
			move_arm = self.MoveIt_left_arm
			rospy.sleep(1)
		elif Handness == 'right':
			self.Gripper_Control(self.Right_Gripper,'open')
			move_arm = self.MoveIt_right_arm
			rospy.sleep(1)	        
		else:
			print('Invalid input parameter for Leave Cube function.')
			raise ValueError

		start_position = self.Get_End_Point_Positon(Handness)
		_, orientation = self.TFlistener.lookupTransform('/base', Handness + '_gripper', rospy.Time(0))

		while True:
			Offsets = raw_input('Please input offsets x, y, z: \n')
			Offsets = map(float, Offsets.split())
			if Offsets[0] == 999:
				break
			end_position = [start_position[0] + Offsets[0], start_position[1] + Offsets[1],start_position[2] + Offsets[2]]
			##MoveIt
			raw_input('Start IK_MoveIt!!!')
			print('start position:', start_position)
			print('end position: ', end_position)
			self.IK_MoveIt(move_arm,rot=orientation, StartPosition=start_position,EndPosition=end_position)	 
			rospy.sleep(0.5)   	    

	def Rotation(self, Handness, radian=0):
		if Handness =='left':
			arm = self.Left_Arm
		elif Handness =='right':
			arm = self.Right_Arm			
		else:
			print('Invalid input parameter for Rotation function.')
			raise ValueError
		while True:
			radian = raw_input('Please enput radian:\n')
			radian = float(radian)
			if radian == 999:
				break
			joint_angles = arm.joint_angles()
			angle = joint_angles[Handness + '_w2']
			angle = self.Trans_Angle(angle + radian)
			joint_angles[Handness + '_w2'] = angle
			self.Move_Joints(Handness, joint_angles)

	def Solve_Cube(self):
		grab_left1, grab_right1 = -0.03, -0.03
		grab_left2, grab_right2 = -0.05, -0.05
		grab_right_offsets1, grab_left_offsets1 = [0,0.02,0.02], [0,0.02,0.02]
		grab_right_offsets2, grab_left_offsets2 = [0, 0, 0], [0, 0, 0]
		opposite_marker = {'red': 'white', 'yellow':'orange', 'blue':'green',
					  'green':'blue', 'white':'red', 'orange':'yellow'}
		front_marker = {'green_white':'yellow', 'orange_white':'green', 'green_orange':'white',
						'white_orange':'blue', 'white_green':'orange', 'green_yellow':'red'}

		def Change_Hands(level='level1',radian=0):
			if self.HoldHand == 'right':
				Hold_Grab_Offset_z = grab_right2
				Hold_Grab_Offsets = grab_right_offsets1
				Hold_Leave_Offsets = [0, 0, 0]
				Rot_Leave_Offsets = [0, 0, 0]
				Rot_Grab_Offset_z = grab_left2
				Rot_Grab_Offsets = [grab_left_offsets1]
			else:
				Hold_Grab_Offset_z = grab_left2
				Hold_Grab_Offsets = grab_left_offsets2
				Hold_Leave_Offsets = [0, 0, 0]
				Rot_Grab_Offset_z = grab_right2
				Rot_Grab_Offsets = [grab_right_offsets2]

			if level == 'level2':
				if self.HoldHand == 'right':
					Rot_Grab_Offsets.append(grab_left_offsets2)
				else:
					Rot_Grab_Offsets.append(grab_right_offsets2)

			marker = self.Get_Marker(self.RotHand) #Initial orientation is always fixed
			self.Grab_Cube(self.RotHand, marker, Marker_z=Rot_Grab_Offset_z, Base_Offsets=Rot_Grab_Offsets[0])
			self.Leave_Cube(self.HoldHand, Hold_Leave_Offsets)
			if len(Rot_Grab_Offsets) == 2:
				marker = self.Get_Marker(self.HoldHand)
				self.Grab_Cube(self.HoldHand, marker, Marker_z=Hold_Grab_Offset_z, Base_Offsets=Hold_Grab_Offsets)
				self.Leave_Cube(self.RotHand, Rot_Leave_Offsets)
				if radian:
					self.Rotation(self.RotHand,radian)
				marker = self.Get_Marker(self.RotHand)
				self.Grab_Cube(self.RotHand, marker, Marker_z=Rot_Grab_Offset_z, Base_Offsets=Rot_Grab_Offsets[1])
				self.Leave_Cube(self.HoldHand, Hold_Leave_Offsets)
			temp = self.HoldHand
			self.HoldHand= self.RotHand
			self.RotHand = temp

		def Rot_Upper_Left(radian): #to rotate upper left
			if self.HoldHand == 'right':
				self.Change_Hands(level='level1')
				marker = self.Get_Marker('right')
				self.Grab_Cube('right', marker, Marker_z=grab_right2, Base_Offsets=grab_right_offsets2)
			else:
				self.Change_Hands(level='level2')
				marker = self.Get_Marker('left')
				self.Grab_Cube('left', marker, Marker_z=grab_left2, Base_Offsets=grab_left_offsets2)

			self.Rotation('right',radian=-radian)
			self.Leave_Cube('right', Offsets=[0,-0.03,-0.03])
			self.Move_Joints('right',right_safe_position)  #We need to record this
			marker = self.Get_Marker('right')
			self.Grab_Cube('right', marker, Marker_z=grab_right2, Base_Offsets=grab_right_offsets2)
			self.Leave_Cube('left',Offsets=[0,0.03,-0.03])
			self.Move_Joints('left',left_safe_position)
			self.HoldHand = 'right'
			self.RotHand = 'left'
		def Rot_Upper_Right(radian):
			if self.HoldHand == 'right':
				self.Change_Hands(level='level2')
				marker = self.Get_Marker('right')
				self.Grab_Cube('right', marker, Marker_z=grab_right2,Base_Offsets=grab_right_offsets2)
			else:
				self.Change_Hands(level='level1')
				marker = self.Get_Marker('left')
				self.Grab_Cube('left', marker, Marker_z=grab_left2, Base_Offsets=grab_left_offsets2)

			self.Rotation('left',radian=-radian)
			self.Leave_Cube('left',Offsets=[0,-0.03,-0.03])
			self.Move_Joints('left', left_safe_position)
			marker = self.Get_Marker('left')
			self.Grab_Cube('left', marker, Marker_z=grab_left2,Base_Offsets=grab_left_offsets2)
			self.Leave_Cube('right', Offsets=[0,-0.03,-0.03])
			self.Move_Joints('right',right_safe_position)
			self.HoldHand = 'left'
			self.RotHand ='right'
		def Rot_Front(radian):
			if self.HoldHand == 'right':
				self.Change_Hands(level='level1')
				marker = self.Get_Marker('right')
				self.Rotation('right', radian=1.575)
				self.Grab_Cube('right', marker, Marker_z=grab_right2)
				self.Leave_Cube('left', Offsets=[0, 0.03, -0.03])
				self.Move_Joints('left', left_safe_position)
				self.Rotation('right', radian=-1.575)
				Rot_Lower_Left(radian)
			else:
				self.Change_Hands(level='level1')
				marker = self.Get_Marker('left')
				self.Rotation('left', radian=1.575)
				self.Grab_Cube('left', marker, Marker_z=grab_left2)
				self.Leave_Cube('right', Offsets=[0,-0.03,-0.03])
				self.Move_Joints('right', right_safe_position)
				self.Rotation('left',radian=1.575)
				Rot_Lower_Right(radian)
		def Rot_Back(radian):
			self.Rotation(self.HoldHand, radian=3.145)
			Rot_Front(radian)
		def Rot_Lower_Left(radian):
			if self.HoldHand == 'left':
				self.Change_Hands('level2')
			marker = self.Get_Marker('left')
			self.Grab_Cube('left', marker, Marker_z=grab_left1)
			self.Rotation('left', radian=-radian)
			self.Leave_Cube('left', Offsets=[0,0.03,-0.03])
			self.Move_Joints('left', left_safe_position)
		def Rot_Lower_Right(radian):
			if self.HoldHand == 'right':
				self.Change_Hands('level2')
			marker = self.Get_Marker('right')
			self.Grab_Cube('right', marker, Marker_z=grab_right1)
			self.Rotation('right', radian=-radian)
			self.Leave_Cube('right', Offsets=[0,-0.03,-0.03])
			self.Move_Joints('right', right_safe_position)
		def Find_Rot_Face(marker):
			if self.LeftMarker == marker: 
				return 'lower_left'
			elif self.RightMarker == marker:
				return 'lower_right'
			elif opposite_marker[self.LeftMarker] == marker:
				return 'upper_right'
			elif opposite_marker[self.RightMarker] == marker:
				return 'upper_left'
			elif marker == front_marker[self.LeftMarker+'_'+self.RightMarker]:
				return 'front'
			return 'back'

		i = 0
		while True:
			if self.Solutions and i < len(self.Solutions):
				number = self.Solutions[i].number
				marker = self.Solutions[i].marker # marker should be string like red, yellow, green 
				radian = self.Solutions[i].radian
				if number == -1:
					print('Finish solving the cube !!! \n Well Done Baxter!')
					break
				face = Find_Rot_Face(marker)
				if face == 'upper_left':
					Rot_Upper_Left(radian)
				elif face == 'upper_right':
					Rot_Upper_Right(radian)
				elif face == 'front':
					Rot_Front(face)
				elif face == 'back':
					Rot_Back(radian)
				elif face == 'lower_left':
					Rot_Lower_Left(radian)
				elif face == 'lower_right':
					Rot_Lower_Right(radian)
				else:
					print('Cannot find correct face, something is wrong!\n')
					break
				i += 1
		self.Move_Joints('left', left_finish_posision)
		self.Move_Joints('right', right_finish_position)

	def Process_Thread(self):
		#Start turn the cube using the algorithm.
		self.Camera_Control('right_hand_camera','open')
		self.Camera_Control('head_camera','open')
		self.Camera_Control('left_hand_camera','close')
		
		def callback(msg): # Start listening solutions
			self.Solutions.append(msg)
		rospy.Subscriber('solutions', solution, callback)

		raw_input('Start Initial Check!!! Press ''Enter'' to go!!!\n')
		self.Initial_Check()
		self.Camera_Control('head','close')
		self.Camera_Control('right','open')
		self.Camera_Control('left','open')
		print('Initial Check completed!')

		while not rospy.is_shutdown():
			if not self._rs.state().enabled:
				rospy.logerr("Control rate timeout.")
			break
			print('\n Starting rotate the Cube.')         
			#For example

			self.Solve_Cube()          

if __name__ == '__main__':

    #define the system imput argument 
	magical_cube = Magic_Cube()
	raw_input('Everything is Ready. Press ''Enter'' to go!!!\n')
	magical_cube.Process_Thread()

    






