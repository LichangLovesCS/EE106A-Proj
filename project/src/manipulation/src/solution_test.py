from manipulation.msg import solution
import rospy

def Send_Solutions(number, marker, radian):
	msg = solution(number, marker, radian)
	pub = rospy.Publisher('solutions', solution, queue_size=10)
	pub.publish(msg)

if __name__ == '__main__':
	rospy.init_node('Master', anonymous=True)
	while True:
		number = raw_input('Please enter a number:\n')
		number = int(number)
		if number == 999:
			break
		marker = raw_input('Please enter a marker name:\n')
		radian = raw_input('Please enter radian:\n')
		radian = float(radian)
		Send_Solutions(number, marker, radian)