#!/usr/bin/env python

from sensor_msgs.msg import LaserScan
import rospy

message_types = ["RAWLASER1","RAWLASER2","RAWLASER3"]
topic_name = "/base_scan"
laser_tf_frame = "base_link"

def create_messages(params, words):
#     0    |     1    |     2     |      3      |        4         |       
# RAWLASER1|laser_type|start_angle|field_of_view|angular_resolution|
#-----------------------------------------------------------------------
#       5      |    6   |       7      |     8      | 9 - 8+num_readings |
# maximum_range|accuracy|remission_mode|num_readings|[range_readings]    |
#-----------------------------------------------------------------------
#  9+num_reading | 10+num_reading - 9+num_reading+num_remissions |
# num_remissions |             [remission values]                |
#-----------------------------------------------------------------------
#10+num_reading+num_remissions|11+num_reading+num_remissions|      last
#        ipc_timestamp        |        ipc_hostname         |logger_timestamp
	
	laser_msg = LaserScan()
	laser_msg.header.seq = create_messages.id
	create_messages.id += 1
	laser_msg.header.frame_id = laser_tf_frame

	a_inc = float(words[4])
	a_min = float(words[2])
	a_max = float(words[2])+float(words[3])

	laser_msg.range_min = 0.
	laser_msg.range_max = float(words[5])
	
	num_readings = int(words[8])
	laser_msg.ranges = list(map(float,words[9:8+num_readings+1]))

	# min-max angle fitting, Karto need
	factor_angle_fitting = a_inc / 2.0
	while (round((a_max - a_min)/a_inc) + 1) != num_readings:
		if (round((a_max - a_min)/a_inc) + 1) > num_readings:
			a_min += factor_angle_fitting
		else:
			a_max -= factor_angle_fitting

		factor_angle_fitting /= 2.0
		
	laser_msg.angle_increment = a_inc
	laser_msg.angle_min = a_min
	laser_msg.angle_max = a_max
	num_remissions = int(words[9+num_readings])
	laser_msg.intensities = list(map(float,words[10+num_readings:9+num_readings+num_remissions+1]))

	laser_msg.header.stamp = rospy.Time( float(words[10+num_readings+num_remissions]) )
	return [(topic_name,laser_msg,laser_msg.header.stamp)]
	
create_messages.id = 0
