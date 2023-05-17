from cv_bridge import CvBridge
bridge = cv_bridge.CvBridge()
image = bridge.imgmsg_to_cv2(msg,encoding='passthrough')
img_back = bridge.cv2_to_imgmsg(result_image)


