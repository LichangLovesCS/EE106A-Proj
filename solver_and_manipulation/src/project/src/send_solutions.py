#!/usr/bin/env python3
# Filename      : send_solutioons.py
# Author        : Gongbo Yang, Lingyao Zhang, Shijie Gao
# Created       : November, 2016
# Last Modified : December, 2016
# Code Reference : https://github.com/adrianliaw/PyCuber

from sys import exit as Die
try:
	import sys
	from video import webcam
except ImportError as err:
	Die(err)
from pycuber import *
from pycuber.solver import CFOPSolver
import numpy as np
import rospy
from project.msg import solution
from sensor_msgs.msg import Image

def Send_Solutions():

	f = np.array([[4, 1, 1], [0, 0, 3], [0, 0, 3]]) #green must be front!
	b = np.array([[0, 0, 5], [1, 1, 1], [1, 1, 1]])
	l =	np.array([[0, 0, 3], [2, 2, 5], [2, 2, 5]]) #left must be white
	r = np.array([[2, 2, 2], [4, 3, 3], [4, 3, 3]])
	u = np.array([[2, 4, 4], [2, 4, 4], [0, 4, 4]])
	d = np.array([[3, 3, 1], [5, 5, 5], [5, 5, 5]])

	rospy.init_node('find_solutions', anonymous=True)
	def callback(msg):
		print('Start Image Processing Program...')
		state = webcam.scan()
		if not state:
			print('\033[0;33m[QBR SCAN ERROR] Ops, you did not scan in all 6 sides.')
			print('Please try again. \033[0m')
			Die(1)
		



	image_info = rospy.Subscriber('/cameras/head_camera/image', Image, callback)

	# f = np.array([[1, 1, 1], [0, 0, 0], [0, 0, 0]]) #green must be front!
	# b = np.array([[0, 0, 0], [1, 1, 1], [1, 1, 1]])
	# l =	np.array([[3, 3, 3], [2, 2, 2], [2, 2, 2]]) #left must be white
	# r = np.array([[2, 2, 2], [3, 3, 3], [3, 3, 3]])
	# u = np.array([[4, 4, 4], [4, 4, 4], [4, 4, 4]])
	# d = np.array([[5, 5, 5], [5, 5, 5], [5, 5, 5]])
	colordic={0:'green', 1:'blue', 2:'white', 3:'yellow', 4:'red', 5:'orange'}

	c = Cube([
		Corner(B=Square(colordic[b[2][2]]),
			   L=Square(colordic[l[2][0]]),
			   D=Square(colordic[d[2][0]])),

		Corner(R=Square(colordic[r[2][0]]),
			   D=Square(colordic[d[0][2]]),
			   F=Square(colordic[f[2][2]])),

		Edge(L=Square(colordic[l[1][2]]),
			 F=Square(colordic[f[1][0]])),

		Edge(U=Square(colordic[u[1][0]]),
			 L=Square(colordic[l[0][1]])),

		Edge(R=Square(colordic[r[1][2]]),
			 B=Square(colordic[b[1][0]])),

		Edge(D=Square(colordic[d[0][1]]),
			 F=Square(colordic[f[2][1]])),

		Edge(R=Square(colordic[r[2][1]]),
			 D=Square(colordic[d[1][2]])),

		Centre(R=Square(colordic[r[1][1]])),

		Centre(U=Square(colordic[u[1][1]])),

		Corner(R=Square(colordic[r[0][2]]),
			   B=Square(colordic[b[0][0]]),
			   U=Square(colordic[u[0][2]])),

		Edge(B=Square(colordic[b[2][1]]),
			 D=Square(colordic[d[2][1]])),

		Corner(L=Square(colordic[l[2][2]]),
			   D=Square(colordic[d[0][0]]),
			   F=Square(colordic[f[2][0]])),

		Corner(U=Square(colordic[u[2][0]]),
			   L=Square(colordic[l[0][2]]),
			   F=Square(colordic[f[0][0]])),

		Edge(R=Square(colordic[r[0][1]]),
			 U=Square(colordic[u[1][2]])),

		Corner(R=Square(colordic[r[0][0]]),
			   U=Square(colordic[u[2][2]]),
			   F=Square(colordic[f[0][2]])),

		Edge(B=Square(colordic[b[1][2]]),
			 L=Square(colordic[l[1][0]])),

		Edge(L=Square(colordic[l[2][1]]),
			 D=Square(colordic[d[1][0]])),

		Centre(L=Square(colordic[l[1][1]])),

		Corner(R=Square(colordic[r[2][2]]),
			   B=Square(colordic[b[2][0]]),
			   D=Square(colordic[d[2][2]])),

		Edge(U=Square(colordic[u[2][1]]),
			 F=Square(colordic[f[0][1]])),

		Centre(F=Square(colordic[f[1][1]])),

		Edge(R=Square(colordic[r[1][0]]),
			 F=Square(colordic[f[1][2]])),

		Corner(B=Square(colordic[b[0][2]]),
			   U=Square(colordic[u[0][0]]),
			   L=Square(colordic[l[0][0]])),

		Centre(D=Square(colordic[d[1][1]])),

		Edge(B=Square(colordic[b[0][1]]),
			 U=Square(colordic[u[0][1]])),

		Centre(B=Square(colordic[b[1][1]]))
		])

	print(c)

	solver = CFOPSolver(c)
	solutions = str(solver.solve(suppress_progress_messages=True))
	length = len(solutions)
	print('This is solution!!!\n' + solutions)
	face_table = {'F': 'green', 'U': 'red', 'L':'white', 'R':'yellow', 'D':'orange', 'B': 'blue'}
	pub_num = 1
	i = 0

	pub = rospy.Publisher('solutions', solution, queue_size=10)
	r = rospy.Rate(10)	
	raw_input('Press enter to continue!!!\n')
	while not rospy.is_shutdown():
		if i >= length:
			pub_num = -1
			pub_msg = solution(number=-1, face=None, radian=None)
			pub.publish(pub_msg)
			break
		cur = solutions[i]
		print('current: ', cur)
		if cur in face_table:
			pub_face = face_table[cur]
			if i+1 < length and solutions[i+1] == "'":
				pub_radian = -1.575
				i += 1
			elif i+1 < length and solutions[i+1] == '2':
				pub_radian = 3.14
				i += 1
			else:
				pub_radian = 1.575
			pub_msg = solution(number=pub_num, face=pub_face, radian=pub_radian)
			pub_num += 1
			pub.publish(pub_msg)

		i += 1
		
		r.sleep()
	print('Finish publishing solutions!!!')

if __name__ == '__main__':
	try:
		Send_Solutions()
	except rospy.ROSInterruptException: 
		pass
