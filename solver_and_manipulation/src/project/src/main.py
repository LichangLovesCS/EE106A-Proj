import sys,argparse,socket,os, thread
from threading import Thread

import tf, moveit_commander
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import copy

from moveit_msgs.msg import *
from geometry_msgs.msg import PoseStamped, Pose
from project.msg import solution

import constant_parameters
from constant_parameters import left_hang, right_hang, cube_table, camera_threads, right_prepare_position
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

		##### Initializa Topic: cube_images #####
		self.pub = rospy.Publisher('cube_images', Image, queue_size=10)

		#set up MoveIt
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()

		self.MoveIt_left_arm = moveit_commander.MoveGroupCommander('left_arm')
		# self.MoveIt_left_arm.set_planner_id('RRTConnectkConfigDefault')
		# self.MoveIt_left_arm.num_planning_attempts(10)
		self.MoveIt_left_arm.set_planning_time(10)
		self.MoveIt_left_arm.set_goal_position_tolerance(0.01)
		self.MoveIt_left_arm.set_goal_orientation_tolerance(0.01) 

		self.MoveIt_right_arm = moveit_commander.MoveGroupCommander('right_arm')
		# self.MoveIt_right_arm.set_planner_id('RRTConnectkConfigDefault')
		# self.MoveIt_left_arm.num_planning_attempts(10)
		self.MoveIt_right_arm.set_planning_time(10)

		self.MoveIt_right_arm.set_goal_position_tolerance(0.01)
		self.MoveIt_right_arm.set_goal_orientation_tolerance(0.01) 
		# moveit_commander.MoveGroupCommander.set_max_velocity_scaling_factor(0.1) 		

		self.Left_Arm = baxter_interface.Limb('left')
		self.Right_Arm = baxter_interface.Limb('right')
		# self.Left_Arm.set_joint_position_speed(0.2)
		# self.Right_Arm.set_joint_position_speed(0.2)

		self.Left_Gripper = baxter_interface.Gripper('left', CHECK_VERSION)
		self.Left_Gripper.set_holding_force(60) 
		self.Left_Gripper.set_dead_band(5)
		self.Left_Gripper.set_moving_force(80)
		# print('Calibrating left gripper...')
		# self.Left_Gripper.calibrate()
		rospy.sleep(0.5)

		self.Right_Gripper = baxter_interface.Gripper('right', CHECK_VERSION)
		self.Right_Gripper.set_holding_force(60) 
		self.Right_Gripper.set_dead_band(5)
		self.Left_Gripper.set_moving_force(80)
		# print('Calibrating right gripper...')
		# self.Right_Gripper.calibrate()
		rospy.sleep(0.5)
		#relative transformation 
		self.TFlistener = tf.TransformListener()

		self.HoldHand = 'left'
		self.RotHand = 'right'
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
			print('Gripper opened')
		elif command == 'close':
			Gripper.close()
			print('Gripper closed')
		else:
			print('The gripper command is not valid')

	def Trans_Angle(self, angle):
		if angle > 3:
			angle = angle - 2 * pi
			print("minus 2 pi----")
		elif angle < -3:
			angle += 2 * pi
			print('plus 2 pi----')
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

		self.Camera_Control('left_hand_camera','close')
		self.Camera_Control('right_hand_camera','open')
		self.Camera_Control('head_camera','open')
		print('############################################################################################3')
		print('Start Initial Check!!! Press ''Enter'' to start calibration...\n')
		# initial state
		left_angles = Joint_Convert(0,'left')
		right_angles = Joint_Convert(0,'right')
		self.Move_Joints('left',left_angles)
		self.Move_Joints('right',right_angles)
		# self.Gripper_Control(self.Left_Gripper, 'open')
		self.Gripper_Control(self.Right_Gripper, 'open')
		rospy.sleep(1.5)

		print('\n Zero position compeleted. Press ''Enter'' to continue...\n')
		
		#prone to get cube
		left_angles = Joint_Convert(1,'left')
		self.Move_Joints('left',left_angles)
		rospy.sleep(0.5)
		#There is a thing that can be optimaized
		raw_input('\n Please place the red side to Left_Gripper and press the enter key to start scan cube.')

		self.Gripper_Control(self.Left_Gripper, 'close')
		rospy.sleep(1)
		
		# Check 1st side
		print('Start checking 1st side ...')
		left_angles = Joint_Convert(2,'left')
		self.Move_Joints('left',left_angles)
		print('Finish checking 1st side')
		rospy.sleep(0.5)
		# raw_input('Send Images')
		# self.Send_Images()
		rospy.sleep(0.5)

		#Check 2nd side
		print('Start checking 2nd side')
		self.Rotation('left',radian=3.14,verbose = False)
		print('Finish checking 2nd side')
		rospy.sleep(0.5)
		# self.Send_Images()
		rospy.sleep(0.5)

		# Check 3rd side
		print('Start checking 3rd side ...')
		left_angles = Joint_Convert(3,'left')
		self.Move_Joints('left',left_angles)
		rospy.sleep(0.5)
		# self.Send_Images()
		rospy.sleep(0.5)
		print('Finish checking 3rd side')

		print('Start checking 4th side ...')
		left_angles = Joint_Convert(4,'left')
		self.Move_Joints('left',left_angles)

		right_angles = Joint_Convert(4,'right')
		self.Move_Joints('right',right_angles)
		# raw_input('111111')
		rospy.sleep(1)
		print('Changing Hand.....')
		_, orientation = self.TFlistener.lookupTransform('/base', 'right_gripper', rospy.Time(0))
		start_position = self.Get_End_Point_Positon('right')

		# middle_position,_ =  self.TFlistener.lookupTransform('/base', self.RightMarker, rospy.Time(0))					
		# while True:
		# 	Base_Offsets = raw_input('Please input Base offsets x, y, z: \n')
		# 	end_position = map(float, Base_Offsets.split())
		# 	if end_position[0] == 999:
		# 		break
		end_position = [0.680, 0, 0.42]
		self.IK_MoveIt(self.MoveIt_right_arm, rot=orientation,StartPosition=start_position,EndPosition=end_position)
		rospy.sleep(0.5)
		print('Changing Hand compeleted!!')

		self.Gripper_Control(self.Right_Gripper,'close')
		rospy.sleep(0.5)
		self.Gripper_Control(self.Left_Gripper,'open')
		rospy.sleep(1)
		
		raw_input('About to move away right arm!!!\n')
		_, right_orientation = self.TFlistener.lookupTransform('/base','right_gripper', rospy.Time(0))
		right_start_position = self.Get_End_Point_Positon('left')	
		right_middle_position = [right_start_position[0], right_start_position[1], right_start_position[2]+0.2]				
		right_end_position = [0.666, -0.133, right_middle_position[2]+0.1]
		self.IK_MoveIt(self.MoveIt_right_arm, rot=right_orientation, StartPosition=right_start_position, MiddlePosition=right_middle_position, EndPosition=right_end_position)

		rospy.sleep(0.5)
		left_angles = Joint_Convert(5,'left')
		self.Move_Joints('left', left_angles)
		right_angles = Joint_Convert(5,'right') # This is equal to x=0.36 y=-0.006, z=0.679, orientaion x=-0.69,y=0.139,z=0.087,w=0.7
		self.Move_Joints('right',right_angles)
		print('Finish checking 4th side')
		rospy.sleep(0.5)
		# self.Send_Images()
		rospy.sleep(0.5)

		print('Start checking 5th side')
		self.Rotation('right',radian=3.14,verbose = False)
		print('Finish checking 5th side')
		rospy.sleep(0.5)
		# self.Send_Images()
		rospy.sleep(0.5)
		
		print('Start checking 6th side.....')
		right_angles = Joint_Convert(6,'right')
		self.Move_Joints('right',right_angles)
		print('Finish checking last side')
		rospy.sleep(0.5)
		# self.Send_Images()
		rospy.sleep(0.5)

		print('#############################################################################################')
		print('Finish Initial Check...')

	def Send_Images(self):
		def callback(msg):
			pass
		rospy.Subscriber('/cameras/head_camera/image', Image, callback)
		while True:
			image_msg = rospy.wait_for_message('/cameras/head_camera/image',Image)
			if image_msg:
				self.pub.publish(image_msg)
				break

	# Move to some place by specifying joint angles
	def Move_Joints(self, Handness, joint_states):
		assert type(joint_states) == dict, 'Joint_States should be dictionary!\n Here are Joint Names: ' + Joint_Names
		# angles = dict(zip(self.joint_names(),[0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]))			
		if Handness =='left':
			self.MoveIt_left_arm.set_joint_value_target(joint_states)
			plan = self.MoveIt_left_arm.plan()
			self.MoveIt_left_arm.execute(plan)
		elif Handness == 'right':
			self.MoveIt_right_arm.set_joint_value_target(joint_states)
			plan = self.MoveIt_right_arm.plan()
			self.MoveIt_right_arm.execute(plan)
		else:
			print('Invalid input parameter for Move_Joints function.')

	def Get_Marker(self, Handness):
		assert Handness=='left' or Handness=='right', 'The Handeness is only "left" or "right"'
		def callback(msg):
			pass
		detected = {}
		topic = '/ar_pose_marker'
		rospy.Subscriber(topic, AlvarMarkers, callback)
		print('start getting marker...')
		while True:
			msg1 = rospy.wait_for_message(topic, AlvarMarkers)
			for marker in msg1.markers:
				detected = marker.id
				if detected and Handness in marker.header.frame_id:
					if 'ar_marker_' + str(detected) in cube_table: 
						# print('Getting marker successful...')
						print('I find marker : ', cube_table['ar_marker_'+str(detected)])
						return 'ar_marker_'+str(detected)  
		# print('finish getting marker...')
				
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
		# Arm.set_max_velocity_scaling_factor(0.2)

		# print(plan)
		Arm.execute(plan) 
		# goal = PoseStamped()
		# goal.header.frame_id = "base"
		# goal.pose.orientation.x = rot[0]
		# goal.pose.orientation.y = rot[1]
		# goal.pose.orientation.z = rot[2]
		# goal.pose.orientation.w = rot[3]
		# goal.pose.position.x = EndPosition[0]
		# goal.pose.position.y = EndPosition[1]
		# goal.pose.position.z = EndPosition[2]
		# Arm.set_pose_target(goal)
		# Arm.set_start_state_to_current_state()
		# plan = Arm.plan()
		# self.Left_Arm.set_joint_velocities(dict(zip(self.Left_Arm.joint_names(),
  #                         [0.1,0.1,0.1,0.1,0.1,0.1,0.1])))
		# self.Right_Arm.set_joint_velocities(dict(zip(self.Right_Arm.joint_names(),
  #                         [0.1,0.1,0.1,0.1,0.1,0.1,0.1])))
		# Arm.execute(plan)

	def Get_End_Point_Positon(self, Handness):
		if Handness == 'left':
			arm = self.Left_Arm
		else:
			arm = self.Right_Arm
		for i in range(50):
			POSE = arm.endpoint_pose()
			EndPoint = [POSE['position'].x ,POSE['position'].y, POSE['position'].z]
		return EndPoint

	def Grab_Cube(self,Handness,AR_marker,Accuracy=0.001,Marker_z=0,Base_Offsets=[0,0,0],verbose = True):
		#There are two direction that can grasp the Arm, one is vertical and another is horizontal.
	
		if Handness =='left':
			move_arm = self.MoveIt_left_arm
			arm = self.Left_Arm
			gripper = self.Left_Gripper
			# self.LeftMarker = cube_table[AR_marker]
		elif Handness == 'right':
			move_arm = self.MoveIt_right_arm
			arm = self.Right_Arm
			gripper = self.Right_Gripper
			# self.RightMarker = cube_table[AR_marker]
		else:
			print('Invalid input parameter for Grab_Cube function.')
		rospy.sleep(2.5)
		self.Gripper_Control(gripper,'open')
		_, orientation = self.TFlistener.lookupTransform('/base', Handness + '_gripper', rospy.Time(0))
		start_position = self.Get_End_Point_Positon(Handness)
		ave_omega, ave_theta = 0, 0
		for i in range(1,20):
			marker_position, rot= self.TFlistener.lookupTransform('/base', AR_marker,rospy.Time(0))								
			omega, theta = eqf.quaternion_to_exp(rot)
			ave_omega = omega + ave_omega
			ave_theta = theta + ave_theta
		omega = ave_omega/len(range(1,20))
		theta = ave_theta/len(range(1,20))
		# print('marker position:', marker_position)
		R1 = eqf.create_rbt(omega, theta, marker_position)
		if verbose == True:
			while True:
				# Marker_z = raw_input('Please input marker offset z: \n')
				# Marker_z = float(Marker_z)
				Base_Offsets = raw_input('Please input Base offsets x, y, z: \n')
				Base_Offsets = map(float, Base_Offsets.split())
				if Base_Offsets[0] == 999:
					break
				R2 = np.matrix([[1,0,0,0],
								[0,1,0,0],
								[0,0,1,Marker_z],
								[0,0,0,1]])
				R = np.matrix(R1) * R2
				end_position = [R[0,3] + Base_Offsets[0] ,R[1,3] + Base_Offsets[1],R[2,3] + Base_Offsets[2]]
				## MoveIt		
				# print('start position:', start_position)
				# print('end position: ', end_position)
				# raw_input('Start rotational!!!')
				# self.IK_MoveIt(move_arm,rot=orientation, StartPosition=start_position,EndPosition=start_position, Accuracy=Accuracy)
				print('Start IK_MoveIt!!!')
				self.IK_MoveIt(move_arm,rot=orientation, StartPosition=start_position,EndPosition=end_position, Accuracy=Accuracy) #MiddlePosition=middle_position,
		else:
			R2 = np.matrix([[1,0,0,0],
							[0,1,0,0],
							[0,0,1,Marker_z],
							[0,0,0,1]])
			R = np.matrix(R1) * R2
			end_position = [R[0,3] + Base_Offsets[0] ,R[1,3] + Base_Offsets[1],R[2,3] + Base_Offsets[2]]
			## MoveIt		
			# print('start position:', start_position)
			# print('end position: ', end_position)
			# raw_input('Start rotational!!!')
			# self.IK_MoveIt(move_arm,rot=orientation, StartPosition=start_position,EndPosition=start_position, Accuracy=Accuracy)
			print('Start IK_MoveIt!!!')
			rospy.sleep(0.5)
			print(Base_Offsets)
			self.IK_MoveIt(move_arm,rot=orientation, StartPosition=start_position,EndPosition=end_position, Accuracy=Accuracy) #MiddlePosition=middle_position,
		print('End Start IK_MoveIt!!! Closing ....')
		rospy.sleep(0.5)
		self.Gripper_Control(gripper, 'close')

	def Fine_Control(self, Handness, Offsets=[0, 0, 0]):
		if Handness == 'left':
			move_arm = self.MoveIt_left_arm
		else:
			move_arm = self.MoveIt_right_arm

		start_position = self.Get_End_Point_Positon(Handness)
		_, orientation = self.TFlistener.lookupTransform('/base', Handness + '_gripper', rospy.Time(0))

		# while True:
		# 	Offsets = raw_input('Please input offsets x, y, z: \n')
		# 	Offsets = map(float, Offsets.split())
		# 	if Offsets[0] == 999:
		# 		break
		end_position = [start_position[0] + Offsets[0], start_position[1] + Offsets[1],start_position[2] + Offsets[2]]
		##MoveIt
		rospy.sleep(1.5) 
		# raw_input('Start IK_MoveIt!!!')
		# print('Offset is : ',Offsets)
		# print('start position:', start_position)
		# print('end position: ', end_position)
		self.IK_MoveIt(move_arm,rot=orientation, StartPosition=start_position, EndPosition=end_position)	 
		# rospy.sleep(0.5)   	    

	def Leave_Cube(self, Handness, Offsets=[0,0,0]):
		if Handness =='left':
			rospy.sleep(0.5)
			self.Gripper_Control(self.Left_Gripper,'open')
			rospy.sleep(1.5)
		elif Handness == 'right':
			rospy.sleep(0.5)
			self.Gripper_Control(self.Right_Gripper,'open')
			rospy.sleep(1.5)	        
		else:
			print('Invalid input parameter for Leave Cube function.')
			raise ValueError

		self.Fine_Control(Handness, Offsets) 	    

	def Rotation(self, Handness, radian=0,verbose = True):
		if Handness =='left':
			arm = self.Left_Arm
		elif Handness =='right':
			arm = self.Right_Arm			
		else:
			print('Invalid input parameter for Rotation function.')
			raise ValueError
		rospy.sleep(0.5)
		if verbose == True:
			while True:
				radian = raw_input('Please enput radian:\n')
				radian = float(radian)
				if radian == 999:
					break
				joint_angles = arm.joint_angles()
				angle = joint_angles[Handness + '_w2']
				angle33 = angle
				print('--------------------------------------------------------------------------')
				print('initial angle : ' + str(round(angle,3)))
				angle = self.Trans_Angle(angle + radian)
				angle11 = angle
				print('converted angles : ' + str(round(angle11,3)))
				joint_angles[Handness + '_w2'] = angle
				self.Move_Joints(Handness, joint_angles)
				rospy.sleep(0.5)
				joint_angles223 = arm.joint_angles()
				angle22222 = joint_angles223[Handness + '_w2']
				print('Competed angle : ' + str(round(angle22222,3)))
				print('andlge difference before converted : ' + str(round(angle22222 - angle33,3)))
				print('andlge difference after converted : ' + str(round(self.Trans_Angle(angle22222 - angle33),3)))
				print('--------------------------------------------------------------------------')
		else:
			joint_angles = arm.joint_angles()
			angle = joint_angles[Handness + '_w2']
			angle33 = angle
			print('--------------------------------------------------------------------------')
			print('initial angle : ' + str(round(angle,3)))
			angle = self.Trans_Angle(angle + radian)
			angle11 = angle
			print('converted angles : ' + str(round(angle11,3)))
			joint_angles[Handness + '_w2'] = angle
			self.Move_Joints(Handness, joint_angles)
			rospy.sleep(0.5)
			joint_angles223 = arm.joint_angles()
			angle22222 = joint_angles223[Handness + '_w2']
			print('Competed angle : ' + str(round(angle22222,3)))
			print('andlge difference before converted : ' + str(round(angle22222 - angle33,3)))
			print('andlge difference after converted : ' + str(round(self.Trans_Angle(angle22222 - angle33),3)))
			print('--------------------------------------------------------------------------')
			while True:
				ang_offset = raw_input('Please enput angular offset :\n')
				ang_offset = float(ang_offset)
				if ang_offset == 999:
					break
				joint_angles = arm.joint_angles()
				angle = joint_angles[Handness + '_w2']
				angle = self.Trans_Angle(angle + ang_offset)
				joint_angles[Handness + '_w2'] = angle
				print('Set angle is :',angle)
				self.Move_Joints(Handness, joint_angles)
				



	def Solve_Cube(self):
		def Joint_Convert(n, Handness):
			assert n>=0 , 'The extracted joint state index must larger or equal zero.'
			joint_angles = constant_parameters.new_joint_states[n]
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
		
		opposite_side = {'orange':'red','yellow':'white','green':'blue','red':'orange','white':'yellow','blue':'green'}
		relative_pos = {'red': None,'yellow':None,'white':None,'blue':None,'orange':None,'green':None}
		check_dic = {}
		work_table ={'red':False, 'yellow':False, 'orange':False, 'white':False, 'green':False,'blue':False}
		left_safe_position = Joint_Convert(8, 'left')
		right_safe_position = Joint_Convert(8, 'right')

		def Change_Hands():

			go_to_safe_position(self.HoldHand)
			go_to_safe_position(self.RotHand)
			print("Go to safe position to change hand!!")
			
			if self.HoldHand == 'right':
				# right_angles = Joint_Convert(11, 'right')
				# left_angles = Joint_Convert(11, 'left')
				# raw_input('about to move right arm!')
				# self.Move_Joints('right',right_angles)
				# raw_input('about to move left arm!')
				# self.Move_Joints('left', left_angles)

				grab_marker_z = 0.01  # green 0.01,
				grab_offsets =[0.01,0,-0.01] #[0,0,0.03] #[0,0,0.01]
				leave_offsets =[0,-0.1,-0.02]	
				n = 3
				left_angles = Joint_Convert(3,'left')
				right_angles = Joint_Convert(3, 'right')

			else:
				# right_angles = Joint_Convert(10, 'right')
				# left_angles = Joint_Convert(10, 'left')
				# raw_input('about to move left arm!')
				# self.Move_Joints('left', left_angles)
				# raw_input('about to move right arm!')
				# self.Move_Joints('right',right_angles)

				grab_marker_z = 0
				grab_offsets = [0.025,0,0.015]
				leave_offsets = [0,0.1,0]
				n = 0
				left_angles = Joint_Convert(0, 'left')
				right_angles = Joint_Convert(0, 'right')

			if self.HoldHand == 'right':
				self.Move_Joints('right', right_angles)
				self.Move_Joints('left', left_angles)
			else:
				self.Move_Joints('left', left_angles)
				self.Move_Joints('right', right_angles)
			# raw_input('hold hand leave...\n')
			# self.Leave_Cube(self.HoldHand, Offsets=leave_offsets)
			# go_to_safe_position(self.HoldHand)
			# if self.RotHand == 'right':
			rospy.sleep(0.5)
			print('Next step to get marker .....')
			print(self.RotHand)
			marker = self.Get_Marker(self.RotHand)
			rospy.sleep(2)
			self.Grab_Cube(self.RotHand, marker,Marker_z=grab_marker_z,Base_Offsets=grab_offsets,verbose= True)
			# else:
			# 	raw_input('move joints!\n')
			# 	left_angles = Joint_Convert(11, 'left')
			# 	self.Move_Joints('left', left_angles)
			# 	rospy.sleep(1)
			# 	self.Left_Gripper.close()
			# 	rospy.sleep(1)

			if self.RotHand == 'right':
				rospy.sleep(1)
				self.Left_Gripper.open()
				rospy.sleep(1)
			else:
				rospy.sleep(1)
				self.Right_Gripper.open()
				rospy.sleep(1)

			temp = self.HoldHand
			self.HoldHand = self.RotHand
			self.RotHand = temp

			print('222222222222222222')
			print(self.RotHand)
			
			print(self.HoldHand)
			# raw_input('333333333333333333')
			rospy.sleep(1.5)
			self.Fine_Control(self.RotHand, Offsets=leave_offsets)
			# self.Leave_Cube(self.HoldHand, Offsets=leave_offsets)

			# raw_input('Next step to move to a comfortable position:\n')
			# self.Fine_Control(self.RotHand)
			# raw_input('next step to leave cube!\n')
			# self.Leave_Cube(self.HoldHand, Offsets=leave_offsets)
			
			# if self.HoldHand == 'right':
			# 	rospy.sleep(1)
			# 	self.Right_Gripper.open()
			# 	# move_arm = self.MoveIt_left_arm
			# 	rospy.sleep(1)
			# else:
			# 	rospy.sleep(1)
			# 	self.Left_Gripper.open()
			# 	# move_arm = self.MoveIt_right_arm
			# 	rospy.sleep(1)
			# leave_angles = Joint_Convert(i, self.HoldHand)
			# self.Move_Joints(self.HoldHand, leave_angles)

			# start_position = self.Get_End_Point_Positon(self.RotHand)
			# _, orientation = self.TFlistener.lookupTransform('/base', self.RotHand + '_gripper', rospy.Time(0))

			# while True:
			# 	Offsets = raw_input('Please input offsets x, y, z: \n')
			# 	Offsets = map(float, Offsets.split())
			# 	if Offsets[0] == 999:
			# 		break
			# 	end_position = [start_position[0] + Offsets[0], start_position[1] + Offsets[1],start_position[2] + Offsets[2]]
			# 	raw_input('Start IK_MoveIt!!!')
			# 	self.IK_MoveIt(move_arm,rot=orientation, StartPosition=start_position,EndPosition=end_position)	 
			# 	rospy.sleep(0.5) 

			

			print('about to go to safe position!\n')
			go_to_safe_position(self.HoldHand)
			go_to_safe_position(self.RotHand)

			if self.HoldHand == 'right':
				n = 6
				self.RightMarker = cube_table[marker]
			else:
				n = 7
				self.LeftMarker = cube_table[marker]
						
			print('about to update table:\n')
			left_angles = Joint_Convert(n,'left')
			right_angles = Joint_Convert(n, 'right')

			if self.HoldHand == 'right':
				self.Move_Joints('right', right_angles)
				self.Move_Joints('left', left_angles)
			else:
				self.Move_Joints('left', left_angles)
				self.Move_Joints('right', right_angles)

			marker = self.Get_Marker(self.RotHand)
			if self.RotHand == 'right':
				self.RightMarker = cube_table[marker]
				print('new right marker: ', self.RightMarker)
			else:
				self.LeftMarker = cube_table[marker]
				print('new left marker: ', self.LeftMarker)
			
			update_table()
			print('go to safe position!\n')
			go_to_safe_position(self.HoldHand)
			go_to_safe_position(self.RotHand)


		def go_to_safe_position(Handness):
			if Handness == 'right':
				self.Move_Joints(Handness, right_safe_position)
			else:
				self.Move_Joints(Handness, left_safe_position)

		def update_table():

			for color in work_table:
				work_table[color] = False
				relative_pos[color] = None
			if self.HoldHand == 'right':
				work_table[opposite_side[self.RightMarker]]=True 
				work_table[opposite_side[self.LeftMarker]]=True
				work_table[self.LeftMarker] = True

				relative_pos[self.RightMarker] = 'right_side'
				relative_pos[opposite_side[self.RightMarker]] = 'left_side'
				relative_pos[self.LeftMarker] = 'front_side'
				relative_pos[opposite_side[self.LeftMarker]] = 'back_side'
				relative_pos[Find_Up_Side(self.LeftMarker,self.RightMarker)] = 'up_side'
				relative_pos[opposite_side[Find_Up_Side(self.LeftMarker,self.RightMarker)]] = 'bottom_side'
			if self.HoldHand == 'left':
				work_table[opposite_side[self.RightMarker]]=True 
				work_table[opposite_side[self.LeftMarker]]=True
				work_table[self.RightMarker] = True

				relative_pos[self.LeftMarker] = 'left_side'
				relative_pos[opposite_side[self.LeftMarker]] = 'right_side'
				relative_pos[self.RightMarker] = 'front_side'
				relative_pos[opposite_side[self.RightMarker]] = 'back_side'
				relative_pos[Find_Up_Side(self.LeftMarker,self.RightMarker)] = 'up_side'
				relative_pos[opposite_side[Find_Up_Side(self.LeftMarker,self.RightMarker)]] = 'bottom_side'

			print(relative_pos)
			# raw_input('We got the relative position of Cube:')
		def Find_Num_Joint(color_side):
			if self.HoldHand == 'right':
				if relative_pos[color_side] in ['right_side','up_side','bottom_side']:
					print('Unable to rotate because Right Holder')
					return False
				rot_side = {'left_side':3, 'front_side':4,'back_side':5}
			elif self.HoldHand == 'left':
				if relative_pos[color_side] in ['left_side','up_side','bottom_side']:
					print('Unable to rotate because Left Holder')
					return False
				rot_side = {'right_side':1, 'front_side':2,'back_side':9}
			print('Find the relative position side: ',relative_pos[color_side])
			return rot_side[relative_pos[color_side]]


		def Find_Up_Side(left, right):
			comb = left+'_'+right
			dic = {'yellow_orange':'blue','orange_white':'blue','white_red':'blue','red_yellow':'blue',
					'white_green':'red','green_yellow':'red','yellow_blue':'red','blue_white':'red',
					'blue_orange':'white','orange_green':'white','green_red':'white','red_blue':'white',
					'blue_yellow':'orange','yellow_green':'orange','green_white':'orange','white_blue':'orange',
					'blue_red':'yellow','red_green':'yellow','green_orange':'yellow','orange_blue':'yellow',
					'orange_yellow':'green','yellow_red':'green','red_white':'green','white_orange':'green'}
			return dic[comb]
		# red:3.3
		def Rot_Cube(n, radian=0):
			if self.HoldHand == 'right':
				right_angles = Joint_Convert(n,'right')
				self.Move_Joints('right',right_angles)
				left_angles = Joint_Convert(n,'left')
				self.Move_Joints('left',left_angles)
			else:
				left_angles = Joint_Convert(n,'left')
				self.Move_Joints('left',left_angles)
				right_angles = Joint_Convert(n,'right')
				self.Move_Joints('right',right_angles)
			print('------side index is :',n)

			## n == 9
			#checked three redians
			grab_marker_z=0.055
			grab_offsets=[0, 0, 0] #[0,0,0.02]
			leave_offsets=[0, -0.2, 0]

			#?checked three redians
			if n == 5: # 1.575, 3.18
				grab_marker_z=0.055
				grab_offsets=[0,0,-0.01]
				leave_offsets=[0, 0.3, -0.1]

			#checked three redians	
			if n == 4: # if radian = 3.14 or -3.14, no stuck 
				grab_marker_z=0.065
				grab_offsets=[0,0,0.005]
				leave_offsets=[0, 0.2, 0]

			#checked three redians
			if n == 3: # if radian = 3.14 , first rotate1.7, then rotate1.6; if radian=1.575, rotate1.6
				grab_marker_z=0.055
				grab_offsets=[0.01,0,-0.005] #[0.02 0 0]
				leave_offsets=[0, 0.15, 0]

			#checked three redians
			if n == 0: # if radian = 3.14 or -3.14, no stuck 
				grab_marker_z=0.02
				grab_offsets=[0.01,0,-0.01]
				leave_offsets=[0, -0.3, 0]

			#checked three redians
			if n == 1:#3.24
				grab_marker_z=0.055
				grab_offsets=[0.01,0,0.02]
				leave_offsets=[0, -0.2, 0]
			#checked three redians
			if n == 2:
				grab_marker_z=0.055#0.08
				grab_offsets=[0, 0, 0.01] #[0,0,0.02]
				leave_offsets=[0, -0.2, 0]



			print('Now you are in rot_cube function\n')
			marker = self.Get_Marker(self.RotHand)
			self.Grab_Cube(self.RotHand, marker,Marker_z=grab_marker_z,Base_Offsets=grab_offsets,verbose = False)
			self.Rotation(self.RotHand, radian=radian,verbose = False)
			self.Leave_Cube(self.RotHand, Offsets=leave_offsets)
			go_to_safe_position(self.HoldHand)
			go_to_safe_position(self.RotHand)


		go_to_safe_position(self.HoldHand)
		go_to_safe_position(self.RotHand)
		self.RightMarker ='red'
		self.LeftMarker = 'orange'
		self.HoldHand = 'right'
		self.RotHand = 'left'

		print('update_table-------------------------\n')
		if self.HoldHand == 'right':
			n = 6
		else:
			n = 7						
		left_angles = Joint_Convert(n,'left')
		right_angles = Joint_Convert(n, 'right')
		if self.HoldHand == 'right':
			self.Move_Joints('left', left_angles)
			self.Move_Joints('right', right_angles)		
		else:
			self.Move_Joints('right', right_angles)
			self.Move_Joints('left', left_angles)
		
		marker = self.Get_Marker(self.RotHand)
		if self.RotHand == 'right':
			self.RightMarker = cube_table[marker]
			print('new right marker: ', self.RightMarker)
		else:
			self.LeftMarker = cube_table[marker]
			print('new left marker: ', self.LeftMarker)
		update_table()
		go_to_safe_position(self.HoldHand)
		go_to_safe_position(self.RotHand)

		i = 0
		# 'red', 'green','blue',
		# 'red',
		sidee = ['orange']
		radiannn = {'red':-1.7, 'green':1.575,'blue':-1.58,'orange':1.575,'yellow':1.575,'white':1.575}
		for side in sidee:
			# Change_Hands()
			###########################################
			print('rotate hand is : ',self.RotHand)
			print('hold hand is : ',self.HoldHand)
			###########################################
			print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n')
			print('Next side : ')
			# radiann = raw_input('Rotate radian : ')
			# radiann = float(radiann)
			n = Find_Num_Joint(side)
			if n == False:
				Change_Hands()
				n = Find_Num_Joint(side)
			print(n)
			ra = radiannn[side]
			Rot_Cube(n,ra)
			# go_to_safe_position(self.HoldHand)
			# go_to_safe_position(self.RotHand)





######################################################### MAIN #########################################################
			# if self.Solutions and i < len(self.Solutions):
			# 	self.Move_Joints('left', left_safe_position)
			# 	self.Move_Joints('right', right_safe_position)
				
			# 	number = self.Solutions[i].number
			# 	color = self.Solutions[i].face 
			# 	radian = self.Solutions[i].radian
			# 	if number == -1:
			# 		print('finish solving cube!\n')
			# 		break

			# 	n = Find_Num_Joint(color)
			# 	if n:
			# 		n -= 1
			# 		print('n:  ', n)
			# 		raw_input('About to rot!!!!')
			# 		print('radian: ', radian)
			# 		Rot_Cube(n, radian=radian)
			# 	else:
			# 		raw_input('About to change hands!!!!')
			# 		Change_Hands()
			# 		n = Find_Num_Joint(color)
			# 		n -= 1
			# 		print('n:  ', n)
			# 		print('radian: ', radian)
			# 		Rot_Cube(n, radian=radian)
			# 	i += 1
			
#########################################################  TEST  #########################################################
			# self.HoldHand = raw_input('Input hold hand:\n')


			# go_to_safe_position(self.HoldHand)
			# go_to_safe_position(self.RotHand)



			# n = input('please input n:\n')
			# right_angles = Joint_Convert(n,'right')
			# left_angles = Joint_Convert(n,'left')
			# # self.Left_Arm.move_to_joint_positions(left_angles, timeout = 15)
			# # self.Right_Arm.move_to_joint_positions(right_angles, timeout = 15)
			# if self.HoldHand == 'left':
			# 	self.Move_Joints('left', left_angles)
			# 	self.Move_Joints('right', right_angles)
			# else:
			# 	self.Move_Joints('right', right_angles)
			# 	self.Move_Joints('left', left_angles)
			# raw_input('stop and about to rotate:\n')
			# Rot_Cube(n)
			

			

			

		

	def Process_Thread(self):
		#Start turn the cube using the algorithm.
		# self.Camera_Control('right_hand_camera','open')
		# self.Camera_Control('head_camera','open')
		# self.Camera_Control('left_hand_camera','close')
		
		def callback(msg): # Start listening solutions
			self.Solutions.append(msg)
		rospy.Subscriber('solutions', solution, callback)

		
		# self.Initial_Check()
		self.Camera_Control('head_camera','close')
		self.Camera_Control('right_hand_camera','open')
		self.Camera_Control('left_hand_camera','open')
		raw_input('Initial Check completed!!!!!!!!!!!!!!!!!!!!!!!!!!!')
		self.Solve_Cube()          

if __name__ == '__main__':

    #define the system imput argument 
	magical_cube = Magic_Cube()
	raw_input('Everything is Ready. Press ''Enter'' to go!!!\n')
	magical_cube.Process_Thread()

    






