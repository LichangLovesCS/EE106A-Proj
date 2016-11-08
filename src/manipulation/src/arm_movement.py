import sys,argparse

import tf, moveit_commander
import rospy
import baxter_interface
import copy

from moveit_msgs.msg import *
from geometry_msgs.msg import PoseStamped

class Magic_Cube(object): 

	def __init__(self,args):

		#initialize a node called arm
		rospy.init_node('arm')
	    #set up MoveIt
	    self.moveit_commander.roscpp_initialize(sys.argv)
	    self.robot = moveit_commander.RobotCommander()
	    self.scene = moveit_commander.PlanningSceneInterface()

	    self.MoveIt_left_arm = moveit_commander.MoveGroupCommander('left_arm')
	    self.MoveIt_left_arm.set_goal_position_tolerance(0.01)
	    self.MoveIt_left_arm.set_goal_orientation_tolerance(0.01) 
	    self.MoveIt_right_arm = moveit_commander.MoveGroupCommander('right_arm')
	    self.MoveIt_right_arm.set_goal_position_tolerance(0.01)
	    self.MoveIt_right_arm.set_goal_orientation_tolerance(0.01) 

	    self.Limb_left = baxter_interface.Limb('left')
	    self.Limb_right = baxter_interface.Limb('right')
	    self.Gripper_left = baxter_interface.Gripper('left', CHECK_VERSION)
	    self.Gripper_left.calibrate()
	    self.Gripper_left.set_holding_force(10) #Here need to be optimized

	    self.Gripper_right = baxter_interface.Gripper('right', CHECK_VERSION)
	    self.Gripper_left.calibrate()
	    self.Gripper_left.set_holding_force(10) #Here need to be optimized

	    #relative transformation 
	    self.relative_tf = tf.TransformListener()

	    #handness to determine which hand to eb controlled 
	    self.Handness = 'left' #it can also be right

		print("Getting robot state... ")
    	self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

    def Gripper_Control(self,Gripper,gripper_state):
    	assert (sate == 'open' or state == 'close'),'The Gripper can only be open or close?'
    	if gripper_state == 'open':
    		Gripper.open() 
    	elif gripper_state == 'close':
    		Gripper.close()

    #This fucntion is used for when Baxter Get close to a side of calibration, it will execute the automatically check.  
    def joint_calibration(self,Handness):
    	if Gripper._on_gripper_state = 'close':
    		if Handness == 'left':
    			Gripper_Control(self.Gripper_left,'open')
    		if Handness == 'right':
    			Gripper_Control(self.Gripper_right,'open')

    def IK_MoveIt(Move_arm,rot,StartPosition, MiddlePosition,EndPosition , Accuracy):
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

	    (plan, fraction) = Move_arm.compute_cartesian_path(
	                               waypoints,   # waypoints to follow
	                               Accuracy,        # eef_step
	                               0.0)         # jump_threshold
	    
	    # Execute the plan         
	    #raw_input('Press Enter to go')            
	    Move_arm.execute(plan) 

	def assign_xyz(arr, xyz):
	    xyz.x = arr[0]
	    xyz.y = arr[1]
	    xyz.z = arr[2]
	    if hasattr(xyz, 'w'):
	        xyz.w = arr[3]
	    return xyz

    def goto(trans, rot, Handness):
    	if Handness == 'right'
	    	planner = self.MoveIt_right_arm
	    else:
	        planner = left_planner

	    goal = PoseStamped()
	    goal.header.frame_id = BASE_FRAME

	    assign_xyz(trans, goal.pose.position)
	    assign_xyz(rot, goal.pose.orientation)
	    # find a plan to get there
	    planner.set_pose_target(goal)
	    planner.set_start_state_to_current_state()
	    plan = planner.plan()
	    # go there
	    planner.execute(plan)
	    rospy.sleep(0.5)

    #Devide the entire motion into three parts. First, gettting close to the 
    def get_close(self,side,Handness):
    	


    #This function is used for approach the Magic Cube approximately
    def approach(self,side,Handness):
    	#Before getting close to the magic cube, make sure that the Grpper is open
    	joint_calibration(Handness)
    	side_trans,side_rot = self.relative_tf.lookupTransform('/base',side,rospy.Time(0))

    	#




    def rotate_cube(self,side,Handness):



    	

    def holder(self,PoseStamped):
    #Here we need PoseStamped in order to make sure that the Magic Cube grasp the correct side



    def callback(move):
    	#执行move的stadegy
    	bb =2








    def First_Grasp(self):
    	Gripper_Control(self.Gripper_left,'open')
    	Joint_State= {'left_e0':-0.2017, 'left_e1':1.1562, 'left_s0':1.5056, 'left_s1':-0.4229, 
                       'left_w0':-2.8785, 'left_w1':-0.8893, 'left_w2':0.3973 }
    	Limb_left.move_to_joint_positions(Joint_State,timeout = 10.0)
    	raw_input('Please place Magica Cube within the left gripper')
    	Gripper_Control(self.Gripper_left,'open')

    	##这里是扫描扫描六个面，然后利用Baxter的Head摄像头来进行图像的读取

	def Process_Thread(self):
		rospy.Subscriber(args.move_topic, MoveMessage, callback) #MoveMessage得自己编

		self.First_Grasp()
		raw_input('Everything is Ready. Press ''Enter'' to go!!!')
		


        while not rospy.is_shutdown():
    		if not self._rs.state().enabled:
        		rospy.logerr("Control rate timeout.")
        	break















if __name__ == '__main__':

    #define the system imput argument 
    desc = 'Arm node is used for manipulation and interaction between two arms.'
    parser.argparse.ArgumentParser(description = desc)
    parser.add_argument('-m','--move_topic',required = True,\
    					defualt ='left_camera',help ='move monitored topic')
    args = parser.parse_args(rospy.myargv(argv=sys.argv))

    magical_cube = Magic_Cube(args)
    magical_cube.Process_Thread()

    






