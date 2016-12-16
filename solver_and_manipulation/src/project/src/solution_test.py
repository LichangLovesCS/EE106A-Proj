#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
from project.msg import solution 

#Import the String message type from the /msg directory of
#the std_msgs package.
from std_msgs.msg import String

#Define the method which contains the main functionality of the node.
def talker():

	#Run this program as a new node in the ROS computation graph 
	#called /talker.
	rospy.init_node('find_solutions', anonymous=True)

	#Create an instance of the rospy.Publisher object which we can 
	#use to publish messages to a topic. This publisher publishes 
	#messages of type std_msgs/String to the topic /chatter_talk
	pub = rospy.Publisher('solutions', solution, queue_size=10)

	# Create a timer object that will sleep long enough to result in
	# a 10Hz publishing rate
	r = rospy.Rate(10) # 10hz

  # Loop until the node is killed with Ctrl-C
	while not rospy.is_shutdown():
    # Construct a string that we want to publish
		pub_number = raw_input('Please enter a number:\n')
		pub_marker = raw_input('Please enter a face:\n')
		pub_radian = raw_input('Please enter radian:\n')

		# Publish our string to the 'chatterer = raw_input('Please enter number')_talk' topic
		pub_msg = solution(number=int(pub_number), face=pub_marker, radian=float(pub_radian))
		pub.publish(pub_msg)

		# Use our rate object to sleep until it is time to publish again
		r.sleep()
		  
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method
	try:
		talker()
	except rospy.ROSInterruptException: pass


# from project.msg import solution
# import rospy
	
# if __name__ == '__main__':
# 	try: 
# 		rospy.init_node('test_solution', anonymous=True)
# 		pub = rospy.Publisher('solutions', solution, queue_size=10)
		
# 		while not rospy.is_shutdown():
# 			number = raw_input('Please enter a number:\n')
# 			number = int(number)
# 			if number == 999:
# 				break
# 			marker = raw_input('Please enter a marker name:\n')
# 			radian = raw_input('Please enter radian:\n')
# 			radian = float(radian)
# 			msg = solution(number=number, marker=marker, radian=radian)
# 			pub.publish(msg)
# 	except rospy.ROSInterruptException: 
# 		print('ERROR!')