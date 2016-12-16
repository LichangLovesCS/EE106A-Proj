''' Image Processing Part'''

def Image_Processing(ROSimage_message):
	cv_msg = cv_bridge.CVBridge.imgmsg_to_cv2(ROSimage_message, desired_encoding='passthrough')