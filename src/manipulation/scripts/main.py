import sys,argparse,socket,os, thread
from threading import Thread

import tf, moveit_commander
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import copy

from moveit_msgs.msg import *
from geometry_msgs.msg import PoseStamped

import constant_parameters
from constant_parameters import Joint_Names,joint_states,loose_position
from ar_track_alvar_msgs.msg import AlvarMarker

from numpy import pi
import math
from sensor_msgs.msg import Image # ROS Image message, Warning!! This is different from OpenCV Image message.
import cv_bridge
import exp_quat_func as eqf
from baxter_core_msgs.srv import (
    ListCameras,
)

loose_position = constant_parameters.loose_position[0]
lse_position = constant_parameters.loose_position[1]

right_hang = {'right_s0':loose_position[11], 'right_s1':loose_position[12], 'right_e0':loose_position[9], 'right_e1':loose_position[10],
                'right_w0':loose_position[13], 'right_w1':loose_position[14], 'right_w2':loose_position[15]}
left_hang = {'left_s0':lse_position[4], 'left_s1':lse_position[5], 'left_e0':lse_position[2], 'left_e1':lse_position[3],
                'left_w0':lse_position[6], 'left_w1':lse_position[7], 'left_w2':lse_position[8]}

camera_threads = {'left_hand_camera':None, 'right_hand_camera':None, 'head_camera':None}

class Magic_Cube(object): 

	def __init__(self):
		print('\nStart Initialization!!!!\n')
		#initialize a node called arm
		rospy.init_node('arm', anonymous=True)

		##### Initialize Camera Information Subscriber #####
		# rospy.init_node('Camera_Iformation_Subscriber', anonymous=True)

		##### AR marker Information #####
		self.cube_table = {'red':'ar_marker_1', 'blue':'ar_marker_2', 'yellow':'ar_marker_3', 
		                  'white':'ar_marker_4', 'orange':'ar_marker_5', 'green': 'ar_marker_6'}
		###### Initialize moveit_commander #####
		moveit_commander.roscpp_initialize('/joint_states:=/robot/joint_states')

		#set up MoveIt
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()

		self.MoveIt_left_arm = moveit_commander.MoveGroupCommander('left_arm')
		self.MoveIt_left_arm.set_goal_position_tolerance(0.01)
		self.MoveIt_left_arm.set_goal_orientation_tolerance(0.01) 
		self.MoveIt_right_arm = moveit_commander.MoveGroupCommander('right_arm')
		self.MoveIt_right_arm.set_goal_position_tolerance(0.01)
		self.MoveIt_right_arm.set_goal_orientation_tolerance(0.01) 

		self.Left_Arm = baxter_interface.Limb('left')
		self.Right_Arm = baxter_interface.Limb('right')

		self.Left_Gripper = baxter_interface.Gripper('left', CHECK_VERSION)
		self.Left_Gripper.set_holding_force(80) 

		self.Right_Gripper = baxter_interface.Gripper('right', CHECK_VERSION)
		self.Right_Gripper.set_holding_force(80) 

		#relative transformation 
		self.TFlistener = tf.TransformListener()

		self.Holder_Arm = self.Left_Arm # default, changing during playing
		self.Holder_Gripper = self.Left_Gripper

		#handness to determine which hand to eb controlled 
		self.Handness = 'left' #it can also be right

		print("Getting robot state... ")
		self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
		self._init_state = self._rs.state().enabled
		print("Enabling robot... ")
		self._rs.enable()
		print("Running. Ctrl-c to quit")

	def Image_Processing(self, ROSimage_message):
		cv_msg = cv_bridge.CVBridge.imgmsg_to_cv2(ROSimage_message, desired_encoding='passthrough')
		### Image Processing Below ###
		##############################

	def Camera_Control(self, camera, command):
		assert (command =='open' or command=='close'), 'only two types of command  are allowed.'
		ls = rospy.ServiceProxy('cameras/list', ListCameras)
		rospy.wait_for_service('cameras/list', timeout=10)
		resp = ls()
		commands = {'left_hand_camera':"Thread(target=self.Parallel_Thread, args=('cd ../launch; roslaunch left_wrist.launch',))",
		            'right_hand_camera': "Thread(target=self.Parallel_Thread, args=('cd ../launch; roslaunch right_wrist.launch',))",
		            'head_camera': "Thread(target=self.Parallel_Thread, args=('cd ../launch; roslaunch head_cam.launch',))"}
		if camera not in resp.cameras:
			print('Cannot control ' + camera + ', other 2 cameras are open. ')
		else:
			if command == 'open':
				if not camera_threads[camera]:
					camera_threads[camera] = eval(commands[camera])
					camera_threads[camera].start()
					print(camera +' is opened.')
				else:
					print(camera + ' has already been launched!')
			elif command == 'close':		
				if camera == 'left_hand_camera':
					camera = baxter_interface.CameraController('left_hand_camera')
				elif camera == 'right_hand_camera':
					camera = baxter_interface.CameraController('right_hand_camera')
				else:
					camera = baxter_interface.CameraController('head_camera')
				camera_threads[camera].join()
				camera_threads[camera] = None
				camera.close()
				print(camera._id+' is closed.')
				
	# The following three functions describe how to control the Gripper 
	def Gripper_Control(self,Gripper,command):
		assert (command == 'open' or command == 'close'),'The Gripper can only be open or close?'
		if command == 'open':
			Gripper.open() 
		elif command == 'close':
			Gripper.close()
		else:
			print('The gripper command is not valid')

	def Trans_Angle(self, angle):
		if angle > pi:
			angle = 2 * pi - angle
		elif angle < - pi:
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
		#initial state
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
		
		#Check 1st side
		print('Checking 1st side ...')
		left_angles = Joint_Convert(2,'left')
		self.Move_Joints('left',left_angles)
		print('Finish checking 1st side')
		rospy.sleep(0.5)

		#Check 2nd side
		raw_input('\n Press enter to continue!')
		print('Checking 2nd side')
		angle = left_angles[self.Left_Arm.name + '_w2'] - pi
		left_angles[self.Left_Arm.name + '_w2'] = self.Trans_Angle(angle)
		self.Move_Joints('left',left_angles)
		print('Finish checking 2nd side')
		rospy.sleep(0.5)

		#Check 3rd side
		raw_input('\n Press enter to continue!')
		print('Checking 3rd side')
		angle = left_angles[self.Left_Arm.name + '_w2'] - pi/2
		left_angles[self.Left_Arm.name + '_w2'] = self.Trans_Angle(angle)
		self.Move_Joints('left',left_angles)
		get_marker = self.Get_Marker('left')

		print('Mission completed, get your marker: ', str(get_marker))
		raw_input('\n Press enter to continue!')

		self.Grab_Cube(self.Right_Arm,self.get_marker,'hori')
		self.Leave_Cube(self.Left_Arm)
		right_angles = Joint_Convert(3,'right')
		self.Move_Joints('right',right_angles)
		print('Finish checking 3rd side')
		rospy.sleep(0.5)


		#Check 4th side
		raw_input('\n Press enter to continue!')
		print('Checking 4th side')
		angle = right_angles[self.Right_Arm.name + '_w2'] + pi/2
		right_angles[self.Right_Arm.name + '_w2'] = self.Trans_Angle(angle)
		self.Move_Joints('right',right_angles)
		print('Finish checking 4th side')
		rospy.sleep(0.5)

		#Check 5th side
		raw_input('\n Press enter to continue!')
		print('Checking 5th side')
		angle = right_angles[self.Right_Arm.name + '_w2'] - pi/2
		right_angles[self.Right_Arm.name + '_w2'] = self.Trans_Angle(angle)
		self.Move_Joints('right',right_angles)
		# get_marker = self.Get_Marker(self.Left_Arm)
		self.Grab_Cube(self.Left_Arm,'vert')
		self.Leave_Cube(self.Right_Arm)
		left_angles = Joint_Convert(4,'right')
		self.Move_Joints('left',left_angles)
		print('Finish checking 5th side')
		rospy.sleep(0.5)

		#Check 6th side
		raw_input('\n Press enter to continue!')
		print('Checking 6th side')
		angle = left_angles[self.Left_Arm.name + '_w2'] + pi
		left_angles[self.Left_Arm.name + '_w2'] = self.Trans_Angle(angle)
		self.Move_Joints('left',left_angles)
		print('Finish checking 6th side')
		rospy.sleep(0.5)

		raw_input('Scan sides finished.')

	# Move to some place by specifying joint angles
	def Move_Joints(self, Handness, joint_states):
		assert type(joint_states) == dict, 'Joint_States should be dictionary!\n Here are Joint Names: ' + Joint_Names
		# angles = dict(zip(self.joint_names(),[0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]))
		print('Got joint states!\n', joint_states, '\n')
		if Handness =='left':
			print(self.Left_Arm.joint_velocities())
			self.Left_Arm.move_to_joint_positions(joint_states, timeout = 15)
		elif Handness == 'right':
			print(self.Right_Arm.joint_velocities())
			self.Right_Arm.move_to_joint_positions(joint_states, timeout = 15)
		else:
			print('Invalid input parameter for Move_Joints function.')

	def Get_Marker(self, Handness):
		#Get Marker`s information
		detected = PoseStamped()
		print('Start getting marker!')
		def call(message):	
			if message.id and Handness in message.header.frame_id:
				detected.header.frame_id ='ar_marker_' + str(message.id)
				print('I have detected ' + detected.header.frame_id)
		while not detected.header.frame_id:
			rospy.Subscriber("ar_pose_marker", AlvarMarker, call)
		return detected.header.frame_id

	def IK_MoveIt(Arm,rot,StartPosition=False, MiddlePosition=False,EndPosition=False , Accuracy=0.03):
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

		(plan, fraction) = Arm.compute_cartesian_path(
		                           waypoints,   # waypoints to follow
		                           Accuracy,        # eef_step
		                           0.0)         # jump_threshold           
		Arm.execute(plan) 

	def Grab_Cube(self,Handness,AR_marker,direction ='hori',Accuracy=0.03,Offset=0.5):
		#There are two direction that can grasp the Arm, one is vertical and another is horizontal.
	
		if Handness =='left':
			self.Gripper_Control(self.Left_Gripper,'open')
			self.Move_Joints(self.Left_Arm,left_hang)
		elif Handness == 'right':
			self.Gripper_Control(self.Right_Gripper,'close')
			self.Move_Joints(self.Left_Arm,right_hang)
		else:
			print('Invalid input parameter for Grab_Cube function.')
		tr,rot = self.TFlistener.lookupTransform('/base', AR_marker,rospy.Time(0))
		omega, theta = eqf.quaternion_to_exp(rot)
		R1 = create_rbt(omega, theta, tr)
		R2 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,Offset],[0,0,0,1]])
		R = R2.dot(R1);
		trans = np.array([R[0][3],R[1][3],R[2][3]])
		if Handness == 'left':
			self.IK_MoveIt(self.Left_Arm, rot=rot, StartPosition=left_hang, MiddlePosition=trans, EndPosition=tr, Accuracy=Accuracy)
			# def IK_MoveIt(Arm,rot,StartPosition=False, MiddlePosition=False,EndPosition=False , Accuracy=0.03):
			self.Gripper_Control(self.Left_Gripper, 'close')
		elif Handness == 'right':
			self.IK_MoveIt(self.Right_Arm,rot=rot, StartPosition=right_hang, MiddlePosition=trans, EndPosition=tr, Accuracy=Accuracy)
			self.Gripper_Control(self.Left_Gripper, 'close')
		else:
			print('Invalid input Handness for Grab_Cube function')

	def Leave_Cube(self,Handness):
	    if Handness =='left':
	        self.Gripper_Control(self.Left_Gripper,'open')
	        self.Move_Joints(self.Left_Arm,left_hang)
	    elif Handness == 'right':
	        self.Gripper_Control(self.Right_Gripper,'close')
	        self.Move_Joints(self.Left_Arm,right_hang)
	    else:
	        print 'Invalid input parameter for Leave Cube function.'

	def Rotation(self,Handness,radian):
	    if Handness =='left':
	        joint_angle = self.Left_Arm.joint_angles
	        joint_angle[self.Left_Arm.name + '_w2'] +=radian
	        self.Move_Joints('left',left_angles)
	    if Handenss =='right':
	        joint_angle = self.Right_Arm.joint_angles
	        joint_angle[self.Right_Arm.name + '_w2'] +=radian
	        self.Move_Joints('right',right_angles)

	def One_Turn(self,target_marker,radian):
	    def find_location(AR_marker,target_marker,Handness):
	        if Handness == 'left':
	            if AR_marker == 'ar_marker_1':
	                m1 = {'ar_marker_2':'upper_left','ar_marker_5':'upper_right',
	                        'ar_marker_3':'front', 'ar_marker_4':'back',
	                        'ar_marker_1':'lower_left','ar_marker_6':'lower_right'}
	            elif AR_marker == 'ar_marker_2':
	                m1 = {'ar_marker_5':'upper_left','ar_marker_6':'upper_right',
	                        'ar_marker_3':'front', 'ar_marker_4':'back',
	                        'ar_marker_2':'lower_left','ar_marker_1':'lower_right'}
	            elif AR_marker == 'ar_marker_3':
	                m1 = {'ar_marker_6':'upper_left','ar_marker_4':'upper_right',
	                        'ar_marker_1':'front', 'ar_marker_5':'back',
	                        'ar_marker_3':'lower_left','ar_marker_2':'lower_right'}
	            elif AR_marker == 'ar_marker_4':
	                m1 = {'ar_marker_2':'upper_left','ar_marker_3':'upper_right',
	                        'ar_marker_1':'front', 'ar_marker_5':'back',
	                        'ar_marker_4':'lower_left','ar_marker_6':'lower_right'}
	            elif AR_marker == 'ar_marker_5':
	                m1 = {'ar_marker_2':'upper_left','ar_marker_4':'upper_right',
	                        'ar_marker_5':'front', 'ar_marker_1':'back',
	                        'ar_marker_3':'lower_left','ar_marker_6':'lower_right'}
	            elif AR_marker == 'ar_marker_6':
	                m1 = {'ar_marker_1':'upper_left','ar_marker_2':'upper_right',
	                        'ar_marker_3':'front', 'ar_marker_4':'back',
	                        'ar_marker_6':'lower_left','ar_marker_5':'lower_right'}
	        elif Handness == 'right':
	            if AR_marker == 'ar_marker_1':
	                m1 = {'ar_marker_6':'upper_left','ar_marker_4':'upper_right',
	                        'ar_marker_1':'front', 'ar_marker_5':'back',
	                        'ar_marker_3':'lower_left','ar_marker_2':'lower_right'}
	            elif AR_marker == 'ar_marker_2':
	                m1 = {'ar_marker_2':'upper_left','ar_marker_4':'upper_right',
	                        'ar_marker_5':'front', 'ar_marker_1':'back',
	                        'ar_marker_3':'lower_left','ar_marker_6':'lower_right'}
	            elif AR_marker == 'ar_marker_3':
	                m1 = {'ar_marker_2':'upper_left','ar_marker_4':'upper_right',
	                        'ar_marker_5':'front', 'ar_marker_1':'back',
	                        'ar_marker_3':'lower_left','ar_marker_6':'lower_right'}
	            elif AR_marker == 'ar_marker_4':
	                m1 = {'ar_marker_1':'upper_left','ar_marker_2':'upper_right',
	                        'ar_marker_3':'front', 'ar_marker_4':'back',
	                        'ar_marker_6':'lower_left','ar_marker_5':'lower_right'}
	            elif AR_marker == 'ar_marker_5':
	                m1 = {'ar_marker_1':'upper_left','ar_marker_2':'upper_right',
	                        'ar_marker_3':'front', 'ar_marker_4':'back',
	                        'ar_marker_6':'lower_left','ar_marker_5':'lower_right'}
	            elif AR_marker == 'ar_marker_6':
	                m1 = {'ar_marker_2':'upper_left','ar_marker_5':'upper_right',
	                        'ar_marker_3':'front', 'ar_marker_4':'back',
	                        'ar_marker_1':'lower_left','ar_marker_6':'lower_right'}
	        return m1
	    def Get_lower_right(position):
	        key_list=[]  
	        value_list=[]  
	        for key,value in position.items():  
	            key_list.append(key)  
	            value_list.append(value)   
	        get_value_index = value_list.index('lower_right')  
	        return key_list[get_value_index]  
	    def Get_lower_left(position):
	        key_list=[]  
	        value_list=[]  
	        for key,value in position.items():  
	            key_list.append(key)  
	            value_list.append(value)   
	        get_value_index = value_list.index('lower_left')  
	        return key_list[get_value_index]

	    if self.Holder_Arm.name =='left':
	        left_marker = self.Get_Marker('left')
	        self.Handness = 'left'
	        position = find_location(left_marker,target_marker,'left')
	        self.Leave_Cube('right')
	        #find the motion state
	        if tar_position =='lower_left':
	            low_right_marker = Get_lower_right(position)
	            self.Grab_Cube('right',low_right_marker)
	            self.Rotation('left',radian)
	            self.Leave_Cube('right')
	        elif tar_position =='lower_right':
	            self.Grab_Cube('right',target_marker)
	            self.Rotation('right',radian)
	            self.Leave_Cube('right')
	        elif tar_position =='upper_left':
	            self.Rotation('left',pi)
	            self.Grab_Cube('right',target_marker)
	            self.Leave_Cube('left')
	            #There`s a bug
	            low_left_marker = Get_lower_right(position)
	            self.Grab_Cube('right',lower_left_marker)
	            self.Rotation('right',radian)
	        elif tar_position =='upper_right':
	            low_right_marker = Get_lower_right(position)
	            self.Grab_Cube('right',low_right_marker)
	            self.Leave_Cube('left')
	            self.Rotation('right',pi)
	            self.Grab_Cube('left',target_position)
	            self.Rotation('left',radian)
	            self.Leave_Cube('right')
	        elif tar_position == 'front':
	            self.Rotation('left',pi/2)
	            #Here has a bug
	            self.Rotation('right',radian)
	            low_left_marker = Get_Marker('left')
	            self.Leave_Cube('left')
	            self.Grab_Cube('left',low_left_marker)
	            self.Leave_Cube('right')
	        elif tar_position =='back':
	            self.Rotation('left',(3*pi)/2)
	            self.Rotation('right',radian)
	            low_left_marker = Get_Marker('left')
	            self.Leave_Cube('left')
	            self.Grab_Cube('left',low_left_marker)
	            self.Leave_Cube('right')
	        else:
	            print 'Invalid target_marker in One_Turn function.'
	    elif self.Holder_Arm.name == 'right':
	        right_makrer = self.Get_Marker('right')
	        self.Handness = 'right'
	        position = find_location(right_makrer,target_marker,'right')
	        self.Leave_Cube('left')
	        if tar_position =='lower_right':
	            low_left_marker = Get_lower_left(position)
	            self.Grab_Cube('left',low_left_marker)
	            self.ROtation('right',radian)
	            self.Leave_Cube('left')
	        elif tar_position == 'lower_left':
	            self.Grab_Cube('left',target_marker)
	            self.Rotation('right',radian)
	            self.Leave_Cube('left')
	        elif tar_position == 'upper_right':
	            self.Rotation('right',pi)
	            self.Grab_Cube('left',target_marker)
	            self.Leave_Cube('right')
	            #There`s must exist a bug
	            low_right_marker = Get_lower_left(position)
	            self.Grab_Cube('left',low_right_marker)
	            self.Rotation('left',radian)
	        elif tar_position =='upper_left':
	            low_left_marker = Get_lower_left(position)
	            self.Grab_Cube('left',low_left_marker)
	            self.Leave_Cube('right')
	            self.Rotation('left',pi)
	            self.Grab_Cube('right',target_position)
	            self.Rotation('right',radian)
	            self.Leave_Cube('left')
	        elif tar_position=='front':
	            self.Rotation('right',pi/2)
	            self.Rotation('left',radian)
	            low_left_marker = Get_Marker('right')
	            self.Leave_Cube('right')
	            self.Grab_Cube('right',low_right_marker)
	            self.Leave_Cube('left')
	        elif tar_position=='back':
	            self.Rotation('right',(3*pi)/2)
	            self.Rotation('left',radian)
	            low_right_marker = Get_Marker('right')
	            self.Leave_Cube('right')
	            self.Grab_Cube('right',low_right_marker)
	            self.Leave_Cube('left')
	
	def Parallel_Thread(self, string):
		os.system(string)
	def Process_Thread(self):
		#rospy.Subscriber(args.move_topic, MoveMessage, callback) 
		raw_input('Everything is Ready. Press ''Enter'' to go!!!')
		#Start turn the cube using the algorithm.
		self.Camera_Control('right_hand_camera','close')
		self.Camera_Control('head_camera','open')
		self.Camera_Control('left_hand_camera','open')
		print('Start Initial Check!!!')
		self.Initial_Check()
	    # self.Camera_Control('head','close')
		# self.Camera_Control('right','open')
		print('mission completed!')

	  #   while not rospy.is_shutdown():
			# if not self._rs.state().enabled:
	  #   		rospy.logerr("Control rate timeout.")
	  #   	break

	         
	  #       print('\n Starting rotate the Cube.')         
	  #       #For example

	  #       self.One_Turn('ar_marker_3',pi/2)          

if __name__ == '__main__':

    #define the system imput argument 
    desc = 'Arm node is used for manipulation and interaction between two arms.'

    # parser.argparse.ArgumentParser(description = desc)
    # parser.add_argument('-m','--move_topic',required = True,\
    # 					defualt ='left_camera',help ='move monitored topic')
    # args = parser.parse_args(rospy.myargv(argv=sys.argv))

    magical_cube = Magic_Cube()
    magical_cube.Process_Thread()

    






