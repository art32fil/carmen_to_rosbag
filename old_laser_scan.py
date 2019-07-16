#!/usr/bin/pyhton

from sensor_msgs.msg import LaserScan
import tf
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point, Quaternion, TransformStamped
import rospy
from math import radians, floor

message_types = ["FLASER","RLASER"]
param_names = {"FLASER":"front_laser","RLASER":"rear_laser"}

tf_topic_name = "/tf"
laser_topic_name = "/base_scan"
laser_tf_child_frame = "base_link"
laser_tf_parrent_frame = "odom"

def create_messages(params, words):
#    0   |      1       | 2 - 1+num_readings | 2+num_readings | 3+num_readings |
# xLASER | num_readings |  [range_readings]  |       x        |        y       |
#-------------------------------------------------------------------------------
# 4+num_readings | 5+num_readings | 6+num_readings | 7+num_readings |
#      theta     |     odom_x     |     odom_y     |   odom_theta   |
#-------------------------------------------------------------------------------
# 8+num_readings | 9+num_readings | 10+num_readings
# ipc_timestamp | ipc_hostname  |logger_timestamp

	laser_msg = LaserScan()
	laser_msg.header.frame_id = laser_tf_child_frame
	
	num_readings = int(words[1])
	laser_msg.ranges = list(map(float,words[2:1+num_readings+1]))
	
	str_laser_fov = "laser_"+param_names[words[0]]+"_fov"
	if str_laser_fov in params:
		a_range = float(params[str_laser_fov][0])
	else:
		a_range = radians(180.0)
	
	str_laser_res = "laser_"+param_names[words[0]]+"_resolution"
	if str_laser_res in params:
		laser_msg.angle_increment = radians(float(params[str_laser_res][0]))
	else:
		laser_msg.angle_increment = a_range / float(num_readings)
	
	laser_msg.angle_min =  -a_range/2.0
	laser_msg.angle_max =   a_range/2.0
	
	str_max_range = "robot_"+param_names[words[0]]+"_max"
	if str_max_range in params:
		laser_msg.range_max = float(params[str_max_range][0])
	else:
		laser_msg.range_max = 20.0
	laser_msg.range_min = 0
	
	laser_msg.header.stamp = rospy.Time(float(words[8+num_readings]))
	
	TS_odom_robot_msg = TransformStamped()
	TS_odom_robot_msg.header.stamp = laser_msg.header.stamp + rospy.Duration(0.01)
	TS_odom_robot_msg.header.frame_id = laser_tf_parrent_frame
	TS_odom_robot_msg.child_frame_id = laser_tf_child_frame
	TS_odom_robot_msg.transform.translation = Point(float(words[5+num_readings]),
	                                                float(words[6+num_readings]), 0.0)
	quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, float(words[7+num_readings]))
	TS_odom_robot_msg.transform.rotation.x  = quaternion[0]
	TS_odom_robot_msg.transform.rotation.y  = quaternion[1]
	TS_odom_robot_msg.transform.rotation.z  = quaternion[2]
	TS_odom_robot_msg.transform.rotation.w  = quaternion[3]
	tf_msg = TFMessage()
	tf_msg.transforms.append(TS_odom_robot_msg)
	
	return [(laser_topic_name, laser_msg, laser_msg.header.stamp),
	        (tf_topic_name, tf_msg, TS_odom_robot_msg.header.stamp)]
