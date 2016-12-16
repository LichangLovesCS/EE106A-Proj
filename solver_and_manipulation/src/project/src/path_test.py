#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
from baxter_interface import gripper as baxter_gripper

def main():
    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Start a node
    rospy.init_node('moveit_node')

    #Initialize both arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    right_arm = moveit_commander.MoveGroupCommander('right_arm')
    left_arm.set_planner_id('RRTConnectkConfigDefault')
    left_arm.set_planning_time(10)
    right_arm.set_planner_id('RRTConnectkConfigDefault')
    right_arm.set_planning_time(10)

    #First goal pose ------------------------------------------------------
    goal_1 = PoseStamped()
    goal_1.header.frame_id = "base"

    #Orientation as a quaternion
    goal_1.pose.orientation.x = 0.271
    goal_1.pose.orientation.y = -0.271
    goal_1.pose.orientation.z = 0.608
    goal_1.pose.orientation.w = 0.694

    while True:
    #x, y, and z position
        position = raw_input('Please input x, y, z:')
        position = map(float, position.split())
        goal_1.pose.position.x = position[0]
        goal_1.pose.position.y = position[1]
        goal_1.pose.position.z = position[2]

        #Set the goal state to the pose you just defined
        left_arm.set_pose_target(goal_1)
        #Set the start state for the left arm
        left_arm.set_start_state_to_current_state()
        #Plan a path
        left_plan = left_arm.plan()
        #Execute the plan
        raw_input('Press <Enter> to move the right arm to goal pose 1 (path constraints are never enforced during this motion): ')
        left_arm.execute(left_plan)

    #Second goal pose -----------------------------------------------------
    rospy.sleep(2.0)
    goal_2 = PoseStamped()
    goal_2.header.frame_id = "base"

    #x, y, and z position
    goal_2.pose.position.x = 0.6
    goal_2.pose.position.y = 0.4
    goal_2.pose.position.z = 0.0
    
    #Orientation as a quaternion
    goal_2.pose.orientation.x = 0.0
    goal_2.pose.orientation.y = -1.0
    goal_2.pose.orientation.z = 0.0
    goal_2.pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    left_arm.set_pose_target(goal_2)

    #Set the start state for the left arm
    left_arm.set_start_state_to_current_state()

    # #Create a path constraint for the arm
    # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "left_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;
    # consts = Constraints()
    # consts.orientation_constraints = [orien_const]
    # left_arm.set_path_constraints(consts)

    #Plan a path
    left_plan = left_arm.plan()

    #Execute the plan
    raw_input('Press <Enter> to move the left arm to goal pose 2: ')
    left_arm.execute(left_plan)


    #Third goal pose -----------------------------------------------------
    rospy.sleep(2.0)
    goal_3 = PoseStamped()
    goal_3.header.frame_id = "base"

    #x, y, and z position
    goal_3.pose.position.x = 0.0
    goal_3.pose.position.y = 0.7
    goal_3.pose.position.z = 0.0
    
    #Orientation as a quaternion
    goal_3.pose.orientation.x = 0.0
    goal_3.pose.orientation.y = -1.0
    goal_3.pose.orientation.z = 0.0
    goal_3.pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    left_arm.set_pose_target(goal_3)

    #Set the start state for the left arm
    left_arm.set_start_state_to_current_state()

    # #Create a path constraint for the arm
    # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "left_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;
    # consts = Constraints()
    # consts.orientation_constraints = [orien_const]
    # left_arm.set_path_constraints(consts)

    #Plan a path
    left_plan = left_arm.plan()

    #Execute the plan
    raw_input('Press <Enter> to move the left arm to goal pose 3: ')
    left_arm.execute(left_plan)

if __name__ == '__main__':
    main()
