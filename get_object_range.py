'''
Lab-3 get_object_range node.

Nimisha Pabbichetty
Prerana Kolipaka
'''

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import CompressedImage, LaserScan
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point,Pose2D
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import math

this_qos_profile = QoSProfile(
	reliability = QoSReliabilityPolicy.BEST_EFFORT,
	history = QoSHistoryPolicy.KEEP_LAST,
	durability = QoSDurabilityPolicy.VOLATILE,
	depth = 5
	)


class Image_lidar_Subscriber(Node):
	def __init__(self):
		super().__init__('image_lidar_subcriber')
		self.pose_sub = self.create_subscription(
			Pose2D,
			'/center_coord',
			self.pose_callback,
			10
			)
			
		self.lidar_sub = self.create_subscription(
			LaserScan,
			'scan',
			self.lidar_callback,
			qos_profile = this_qos_profile)
		self.pose = Pose2D()
		self.publish_dist_angle = self.create_publisher(
		Pose2D,
		'obj_dist_angle',
		10)
		
		#self.ts.registerCallback(self.callback)
			
		
	def pose_callback(self,pose_msg):
		#self.get_logger().info('in pose callback')
		self.pose = pose_msg
		'''
		queue_size = 30
		# you can use ApproximateTimeSynchronizer if msgs dont have exactly the same timestamp
		self.ts = ApproximateTimeSynchronizer(
		[self.pose_sub, self.lidar_sub],
		queue_size,
		0.01,  # defines the delay (in seconds) with which messages can be synchronized
		)	
		'''

	def lidar_callback(self,lidar_msg):
	# do your stuff here
		#elf.get_logger().info('in lidar callback')
		#self.get_logger().info('in callback "%s"', lidar_msg.header)
		pos_rad = 0
		#self.get_logger().info('in lidar callback, pose value from detect_object "%f"'%(self.pose.theta))
		#self.get_logger().info('in lidar callback, ranges length "%f"'%(len(lidar_msg.ranges)))
		distance_list = []
		
		if self.pose.x !=0.0 and self.pose.y != 0.0 and self.pose.theta !=0.0:
			deg_ranges = [self.pose.theta -6, self.pose.theta - 5,self.pose.theta -4,self.pose.theta -3,self.pose.theta -2,self.pose.theta -1,self.pose.theta,self.pose.theta +1,self.pose.theta +2,self.pose.theta +3,self.pose.theta +4,self.pose.theta +5,self.pose.theta +6]
			for i in deg_ranges:
				if i > 0 :
					pos_rad  = 6.283 - (i*0.0174)
				elif i < 0:
					pos_rad = abs(i*0.0174)
				else:
					pos_rad =0.0
			
				range_index = int(pos_rad/lidar_msg.angle_increment)%len(lidar_msg.ranges)
				
				#self.get_logger().info('range index "%f"'%(range_index))
				distance = lidar_msg.ranges[range_index]
				if (math.isnan(distance)!= True):
					distance_list.append(distance)
				
			
			lidar_publish_msg = Pose2D()
			dist = sum(distance_list)/len(distance_list)
			lidar_publish_msg.x = dist
			lidar_publish_msg.theta = pos_rad
			lidar_publish_msg.y = 0.0
			self.get_logger().info('in lidar callback "%f"'%(dist))
			self.publish_dist_angle.publish(lidar_publish_msg)
		else:
			lidar_publish_msg = Pose2D()
			#dist = sum(distance_list)/len(distance_list)
			lidar_publish_msg.x = 0.0
			lidar_publish_msg.theta = 0.0
			lidar_publish_msg.y = 0.0
			#self.get_logger().info('in lidar callback "%f"'%(dist))
			self.publish_dist_angle.publish(lidar_publish_msg)
			
		
		
		
def main(args=None):
	rclpy.init(args=args)

	image_lidar_subscriber = Image_lidar_Subscriber()

	rclpy.spin(image_lidar_subscriber)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	image_lidar_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
    main()
