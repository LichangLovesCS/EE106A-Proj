import sys
import rospy, tf

import moveit_commander
import moveit_msgs.msg, geometry_msgs.msg

import baxter_interface
from baxter_interface import CHECK_VERSION

import constant_parameters
from constant_parameters import Joint_Names


class PlayCube():
    def __init__(self):
    	print('\nStart Initialization!!!!\n')
		##### Initialize AR_tag transforms #####
        rospy.init_node('master', anonymous=True)
        self.listener = tf.TransformListener()

		###### Initialize moveit_commander #####
        moveit_commander.roscpp_initialize('/joint_states:=/robot/joint_states')

        ###### control constant parameters #####
        
        self.count = 0
        ###### Initialize MoveIt for both arms #####
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.MoveIt_left_arm = moveit_commander.MoveGroupCommander('left_arm')
        self.MoveIt_left_arm.set_goal_position_tolerance(0.01)
        self.MoveIt_left_arm.set_goal_orientation_tolerance(0.01) 
        self.MoveIt_right_arm = moveit_commander.MoveGroupCommander('right_arm')
        self.MoveIt_right_arm.set_goal_position_tolerance(0.01)
        self.MoveIt_right_arm.set_goal_orientation_tolerance(0.01) 

        ##### Initialize Gripper control #####
        self.Left_Arm = baxter_interface.Limb('left')
        self.Right_Arm = baxter_interface.Limb('right')
        
        self.Left_Gripper = baxter_interface.Gripper('left', CHECK_VERSION)
        self.Right_Gripper = baxter_interface.Gripper('right', CHECK_VERSION)
        self.Left_Gripper.set_holding_force(70)
        self.Right_Gripper.set_holding_force(70)

        ##### Set Camera #####
        self.Head_Camera = baxter_interface.CameraController('head_camera')
        self.Left_Hand_Camera = baxter_interface.CameraController('left_hand_camera')
        self.Right_Hand_Camera = baxter_interface.CameraController('right_hand_camera')

		##### Verify robot is enabled  #####
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

        print('\nFinish Initialization!!!\n')



    #move left gripper to a previously tested position, parameters from constant_parameters.joint_states
    def Move_Left_Arm(self, goal_joint_states):
        self.Left_Arm.move_to_joint_positions(goal_joint_states, timeout=15.0, threshold=0.008726646)

    #move right gripper to a previously tested position, parameters from constant_parameters.joint_states
    def Move_Right_Arm(self, goal_joint_states):
    	self.Right_Arm.move_to_joint_positions(goal_joint_states, timeout=15.0, threshold=0.008726646)

    def Gripper_Control(self, Gripper, command):
    	assert (command == 'open' or command == 'close'), 'The Gripper can only be open or close!'
        if command == 'open':
           Gripper.open()                  
        elif command == 'close':
            Gripper.close()
        else:
            print('The gripper command is not valid')

    
    # Move to some place by specifying joint angles
    def Move_Joints(self, joint_states, Arm, timeout=15):
    	assert type(joint_states) == dict, 'Joint_States should be dictionary!\n Here are Joint Names: ' + Joint_Names
    	# angles = dict(zip(self.joint_names(),
        #                      [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]))
        print('I got your joint states!\n', joint_states, '\n')
        Arm.move_to_joint_positions(joint_states, timeout)


    #AR_marker specifies which face to grab. Gripper specifies which gripper to use.
    def Grab_Cube(self, Arm, AR_marker, Accuracy=0.03):
        (trans,rot) = self.listener.lookupTransform('/base', AR_marker,rospy.Time(0))

        Goal_Pose = Pose()
        Goal = []
         #Our orientation here should be fixed. We need to test it.
        Goal_Pose.orientation.x = 0
        Goal_Pose.orientation.y = 0
        Goal_Pose.orientation.z = 0
        Goal_Pose.orientation.w = 1

        #we need to add some constant offset here
        Goal_Pose.position.x = trans[0]
        Goal_Pose.position.y = trans[1]
        Goal_Pose.position.z = trans[2]
        Goal.append(Goal_Pose)
        (plan, fraction) = Arm.compute_cartesian_path(
                               Goal,   # waypoints to follow
                               Accuracy,        # eef_step
                               0.0)     
        Arm.execute(plan)

    # Control Camera, camera should be one of the cameras specified in __init__ function, line46
    def Control_Camera(self, camera, command):
    	assert (command == 'open' or command == 'close'), 'The Camera can only be open or close!'
    	if command == 'open':
    		camera.open()
    	else:
    		camera.close()
    	print('Camera Control Finished!\n Command: ' + command + '\n')


        

#######################Start playing now!!!!#######################
    def Start(self):
    	####  1. Initialize states #### 	

        while True:
    	    command = raw_input('COMMAND REQUIRED: ')
            if command == '1':
                joint_angles = constant_parameters.joint_states[0] #Change this part
                joint_angles = {'left_s0':joint_angles[4], 'left_s1':joint_angles[5], 'left_e0':joint_angles[2], 'left_e1':joint_angles[3],
                'left_w0':joint_angles[6], 'left_w1':joint_angles[7], 'left_w2':joint_angles[8]}
                # joint_angles = self.Left_Arm.joint_angles()
                # joint_angles['left_w2'] += 0.2
                self.Move_Joints(joint_angles, self.Left_Arm)
                print('mission complete!')
                print('current joint angles: ', self.Left_Arm.joint_angles())
                camera_command = raw_input('Controling Camera, Please enter open or close: \n')
                which_camera = raw_input('Please choose which camera to control: left, right, head \n')
                if which_camera == 'left':
                	which_camera = self.Left_Hand_Camera
                elif which_camera == 'right':
                	which_camera = self.Right_Hand_Camera
                else:
                	which_camera = self.Head_Camera
                self.Control_Camera(which_camera, camera_command)
            else:
		    	continue
    	####  2.   Observe Cube    ####
    	  ##2.1 get first 3 faces with head camera##
    	  ##2.2 get other 3 faces with head camera##

    	####  3. Calculate Steps   ####

    	####  4.  Planning Motion  ####

if __name__ == "__main__":
	PC = PlayCube()
	PC.Start()



