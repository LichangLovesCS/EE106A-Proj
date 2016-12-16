from sensor_msgs.msg import Image
import rospy
def Send_Images():
	def callback(msg):
		pass
	pub = rospy.Publisher('cube_images', Image, queue_size=10)
	rospy.Subscriber('/usb_cam/image_raw', Image, callback)
	while True:
		image_msg= rospy.wait_for_message('/usb_cam/image_raw',Image)
		if image_msg:
			pub.publish(image_msg)
			break

if __name__ == '__main__':
	rospy.init_node('Master', anonymous=True)
	while True:
		s = raw_input('Please enter a number: ')
		if float(s) == 999:
			break
		Send_Images()