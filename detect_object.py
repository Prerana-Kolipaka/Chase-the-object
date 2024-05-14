'''
Lab-3 detect_object node.

Nimisha Pabbichetty
Prerana Kolipaka
'''

#Import necessary modules
import cv2
import numpy as np
import imutils
    
from sensor_msgs.msg import Image, CompressedImage
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose2D
from std_msgs.msg import String
from cv_bridge import CvBridge


from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

this_qos_profile = QoSProfile(
	reliability = QoSReliabilityPolicy.BEST_EFFORT,
	history = QoSHistoryPolicy.KEEP_LAST,
	durability = QoSDurabilityPolicy.VOLATILE,
	depth = 5
	)
class ImageSubscriber(Node):

	def __init__(self):
		super().__init__('image_subcriber')
		self.subscription = self.create_subscription(
			CompressedImage,
			'image_raw/compressed',
			self.listener_callback,
			qos_profile = this_qos_profile)
			
		self.publish_coord = self.create_publisher(
			Pose2D,
			'center_coord',
			10)
		self.subscription  # prevent unused variable warning
		self.publish_coord
		
	def listener_callback(self, msg):
		#self.get_logger().info('I heard: "%s"' % msg.data)

		#img = msg.data
		#Color threshold for the color of the disc (yellow)
		bridge = CvBridge()
		#image_cv = bridge.imgmsg_to_cv2(msg.data, desired_encoding='passthrough')
		image_cv = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
		low_threshold   = np.array(( 20,100, 100), dtype=np.uint8, ndmin=1)
		high_threshold  = np.array(( 30,255,255),dtype=np.uint8, ndmin=1)

		#while(True):
			#reading frames from laptop camera
			#ret,frame = img.read()
			#converting to HSV for color thresholding
		#img_arr = np.fromstring(msg.data, np.uint8)
		#image_cv = cv2.imdecode(img_arr,cv2.CV_LOAD_IMAGE_COLOR)
		hsv = cv2.cvtColor(image_cv,cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv,low_threshold,high_threshold)
		#cv2.imshow("mask", mask)
			
		#Erode and Dilate to cover the noise
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)
			
			#finding contours and centers
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)
			
			#print centers on terminal
		#     print("Center",cnts)
		center = None
		
		#self.get_logger().info('Reached breakpoint: ')

		center_coord = Pose2D()
		center_coord.x = 0.0
		center_coord.y = 0.0
		center_coord.theta = 0.0
		
		if len(cnts) > 0:
			#finding the largest contour in the masked image, then computing the minimum radius circle enclosing the contour 
			#and the centroid
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			
			center_coord.x = float(center[0])
			center_coord.y = float(center[1])
			center_coord.theta = center_coord.x*0.194 - 31.1
						
		self.publish_coord.publish(center_coord)

def main(args=None):
	rclpy.init(args=args)

	image_subscriber = ImageSubscriber()

	rclpy.spin(image_subscriber)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	image_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
    main()
