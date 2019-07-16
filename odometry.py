#!/usr/bin/python

import tf
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point, Quaternion, TransformStamped
import rospy

message_types = ["ODOM"]
topic_name = "/tf"
laser_tf_child_frame = "base_link"
laser_tf_parrent_frame = "odom"

def create_messages(params, words):
#    0 | 1 | 2 |  3  | 4 | 5 |  6  |      7      |      8     |        9
#  ODOM| x | y |theta| tv| rv|accel|ipc_timestamp|ipc_hostname|logger_timestamp
	TS_odom_robot_msg = TransformStamped()
	TS_odom_robot_msg.header.stamp          = rospy.Time( float(words[7]) )
	TS_odom_robot_msg.header.frame_id       = laser_tf_parrent_frame
	TS_odom_robot_msg.child_frame_id        = laser_tf_child_frame
	TS_odom_robot_msg.transform.translation = Point(float(words[1]), float(words[2]), 0.0)
	quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, float(words[3]))
	TS_odom_robot_msg.transform.rotation.x  = quaternion[0]
	TS_odom_robot_msg.transform.rotation.y  = quaternion[1]
	TS_odom_robot_msg.transform.rotation.z  = quaternion[2]
	TS_odom_robot_msg.transform.rotation.w  = quaternion[3]
	tf_message = TFMessage()
	tf_message.transforms.append(TS_odom_robot_msg)
	return [(topic_name,tf_message,TS_odom_robot_msg.header.stamp)]
