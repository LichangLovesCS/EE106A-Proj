#!/usr/bin/env python
import rospy
from baxter_interface import gripper as baxter_gripper
import sys
rospy.init_node('gripper_control')

#Set up the left gripper
left_gripper = baxter_gripper.Gripper('left')
right_gripper = baxter_gripper.Gripper('right')
#Calibrate the gripper (other commands won't work unless you do this first)
print('Calibrating left gripper...')
left_gripper.calibrate()
rospy.sleep(2.0)

print('Calibrating right gripper...')
right_gripper.calibrate()
rospy.sleep(2.0)

done = False
while not done and not rospy.is_shutdown():
	gripper = raw_input('Choose which griper to control(left or right): \n')
	if gripper == 'left':
		gripper = left_gripper
	else:
		gripper = right_gripper
	control = raw_input('Controlling the gripper: 1 for open, 2 for close\n ')
	gripper.set_holding_force(100)
	if control == '1':
		gripper.open(block=True)
		rospy.sleep(1.0)
		print('OPEN Done!\n')
	elif control == '2':
		gripper.close(block=True)
		rospy.sleep(1.0)
		print('CLOSE Done!\n')				
	else:
		print('Please enter the right command!\n')
