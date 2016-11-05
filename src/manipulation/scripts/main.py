import sys
import rospy, tf

import moveit_commander
import moveit_msgs.msg, geometry_msgs.msg

import baxter_interface
from baxter_interface import CHECK_VERSION

import constant_parameters

class PlayCube():
	def __init__(self):

		##### Initialize AR_tag transforms #####
        rospy.init_node('master', anonymous=True)
        self.listener = tf.TransformListener()

		#Initialize moveit_commander
        moveit_commander.roscpp_initialize('/joint_states:=/robot/joint_states')

        #control constant parameters
        
        self.count = 0
        #Initialize MoveIt for both arms
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.MoveIt_left_arm = moveit_commander.MoveGroupCommander('left_arm')
        self.MoveIt_left_arm.set_goal_position_tolerance(0.01)
        self.MoveIt_left_arm.set_goal_orientation_tolerance(0.01) 
        self.MoveIt_right_arm = moveit_commander.MoveGroupCommander('right_arm')
        self.MoveIt_right_arm.set_goal_position_tolerance(0.01)
        self.MoveIt_right_arm.set_goal_orientation_tolerance(0.01) 

        ##### Initialize Gripper control #####
        self.Arm_left = baxter_interface.Limb('left')
        self.Arm_right = baxter_interface.Limb('right')
        
        self.Left_Gripper = baxter_interface.Gripper('left', CHECK_VERSION)
        self.Right_Gripper = baxter_interface.Gripper('right', CHECK_VERSION)



    #move left gripper to a previously tested position, parameters from constant_parameters.joint_states
    def Move_Left_Arm(self, goal_joint_states):
    	self.Arm_left.move_to_joint_positions(goal_joint_states, timeout=15.0, threshold=0.008726646)

    #move right gripper to a previously tested position, parameters from constant_parameters.joint_states
    def Move_Right_Arm(self, goal_joint_states):
    	self.Arm_left.move_to_joint_positions(goal_joint_states, timeout=15.0, threshold=0.008726646)

    def Gripper_Control(self,Gripper,command):
    
        if command == 'open':
           Gripper.open()                  
        elif command == 'close':
            Gripper.close()
        else:
            print('The gripper command is not valid')

    #AR_marker specifies which face to grab. Gripper specifies which gripper to use.
    def Grab_Cube(self, arm, AR_marker, Accuracy=0.03):
      	(trans,rot) = self.listener.lookupTransform('/base', AR_marker,rospy.Time(0))

        Goal_Pose = Pose()
        Goal = []
         #Our orientation here should be fixed. We need to test it.
        Goal_Pose.orientation.x = 
        Goal_Pose.orientation.y = 
        Goal_Pose.orientation.z = 
        Goal_Pose.orientation.w = 

        #we need to add some constant offset here
        Goal_Pose.position.x = trans[0]
        Goal_Pose.position.y = trans[1]
        Goal_Pose.position.z = trans[2]
        Goal.append(Goal_Pose)
        (plan, fraction) = arm.compute_cartesian_path(
                               Goal,   # waypoints to follow
                               Accuracy,        # eef_step
                               0.0)     
        arm.execute(plan)
        

#######################Start playing now!!!!#######################
    def Start():
    	####  1. Initialize states ####
    	joint_angles = constant_parameters.joint_states[0] #Change this part

    	####  2.   Observe Cube    ####
    	  ##2.1 get first 3 faces with head camera##
    	  ##2.2 get other 3 faces with head camera##

    	####  3. Calculate Steps   ####

    	####  4.  Planning Motion  ####


if __name__ == "__main__":
	PC = PlayCube() 
	PC.Start()



