'''
Lab-3 chase_object node.

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
from geometry_msgs.msg import Point,Pose2D, Twist
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import time


class Chase_Object(Node):

	def __init__(self):
		super().__init__('chase_object')
		self.pose1_sub = self.create_subscription(
		Pose2D,
		'/obj_dist_angle',
		self.control_callback,
		10)
		
		self.vel_pub = self.create_publisher(
		Twist,
		'/cmd_vel',
		10)
		
		self.pose1_sub
		self.vel_pub
		self.desired_distance = 0.6
		self.last_time = None
		self.prev_p_dist_error = 0
		self.desired_angle = 0.0
		self.prev_angle_error = 0
		 
	def control_callback(self, pose_msg):
		
		vel = Twist()
		if pose_msg.x == 0:
	
			vel.linear.x = 0.0
			vel.linear.y = 0.0
			vel.linear.z = 0.0
			vel.angular.x = 0.0
			vel.angular.y = 0.0
			vel.angular.z = 0.0	
			
		else: 
			
			#if dt is None:
			dt=0
			cur_time = time.time()
			if self.last_time is None:
				self.last_time = cur_time
				
			dt = cur_time - self.last_time
			self.last_time = cur_time
				
			curr_p_dist_error =  pose_msg.x - self.desired_distance
			
			
			curr_p_angle_error = min(pose_msg.theta, 6.283-pose_msg.theta)
					     #pose_msg.theta - self.desired_angle 
			
			if(pose_msg.theta<3.14):
				direction = +1 
			else:
				direction = -1
				
			if abs(curr_p_dist_error) <= 0.1:
				curr_p_dist_error = 0
			if curr_p_angle_error <= 0.2:
				curr_p_angle_error = 0
				
			if dt != 0:
				d_dist_error = (curr_p_dist_error- self.prev_p_dist_error)/dt	
				d_angle_error = (curr_p_angle_error - self.prev_angle_error) / dt
				
				kp_dist = 0.2
				kd_dist = 0.1
				
							
				kp_angle = 1
				kd_angle = 0.05
				
				dist_d_term = kd_dist *d_dist_error
				dist_p_term = kp_dist*curr_p_dist_error
				
				angle_d_term = kd_angle * d_angle_error
				angle_p_term = kp_angle * curr_p_angle_error

				#l_v = kp_dist*(curr_p_error)
				l_v = dist_d_term + dist_p_term
				a_v = angle_d_term + angle_p_term
				
				
				self.get_logger().info('in control_callback, err: "%f"' %(curr_p_dist_error))
				
		
				vel.linear.x = min(0.15,l_v)
				vel.linear.y = 0.0
				vel.linear.z = 0.0
				vel.angular.x = 0.0
				vel.angular.y = 0.0
				vel.angular.z = direction*min(a_v,2.84)
				
				self.get_logger().info('in control_callback, vel: "%f"' %(vel.angular.z))
			else:
				
				vel.linear.x = 0.0
				vel.linear.y = 0.0
				vel.linear.z = 0.0
				vel.angular.x = 0.0
				vel.angular.y = 0.0
				vel.angular.z = 0.0
		
			self.prev_p_dist_error = curr_p_dist_error
			self.prev_angle_error = curr_p_angle_error
		
			
		self.vel_pub.publish(vel)
		
		
		


def main(args=None):
	rclpy.init(args=args)

	chase_object = Chase_Object()

	rclpy.spin(chase_object)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	chase_object.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
    main()
	
