#!/usr/bin/env python

import rospy
import roslib
import tf
import numpy as np
import sys, getopt
from geometry_msgs.msg import Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

import os.path
import rosbag

def help_str():
	return "usage: rosrun carmen_publisher custom_creator.py -i path/to/input.log -o path/to/write/rosbag.bag [names_of_function_files_in_same_dir]"

class bag_creator:
	def __init__(self):
		self.modules = []

	def set_input_file(self, inputfile):
		try:
			inCompleteName = os.path.expanduser(inputfile)
			self.f = open(inCompleteName, "r")
			rospy.loginfo("Reading data from : %s", inCompleteName)
		except (IOError, ValueError):
			rospy.logerr("Couldn't open %", inputfile)
			rospy.loerr(help_str())
			rospy.logerr("Job's interrupted. Exiting...")
			exit(-1)
		return self.f
	
	def set_output_file(self, outputfile):
		try:
			outCompleteName = os.path.expanduser(outputfile)
			self.bag = rosbag.Bag(outCompleteName, "w")
			rospy.loginfo("Writing rosbag to : %s", outCompleteName)
		except (IOError, ValueError):
			rospy.logerr("Couldn't open %", outputfile)
			rospy.loerr(help_str())
			rospy.logerr("Job's interrupted. Exiting...")
			exit(-1)
		return self.bag
		
	def set_function_files(self, functionfiles):
		for functionfile in functionfiles:
			try:
				self.modules.append(__import__(functionfile.replace('.py', '')))
			except (ImportError, ValueError):
				rospy.logerr("Couldn't import %s", functionfile)
				rospy.logerr(help_str())
				rospy.logerr("Job's interrupted. Exiting...")
				exit(-1)
		return self.modules
	
	def convert(self, inputfile, outputfile, functionfiles):
		self.set_input_file(inputfile)
		self.set_output_file(outputfile)
		self.set_function_files(functionfiles)
		rospy.loginfo("Start convertion to rosbag")
		i = 1
		found_message_types = {}
		for line in self.f:
			#rospy.loginfo("execute line %d",i)
			i += 1
			words = line.split()
			if words[0][0] == '#':
				continue
			message_type = words[0]
			if message_type not in found_message_types:
				found_message_types[message_type] = False
			for module in self.modules:
				if message_type in module.message_types:
					found_message_types[message_type] = True
					tupple_topic_message_time = module.create_messages(words[1:])
					for topic, message, stamp in tupple_topic_message_time:
						self.bag.write(topic, message, stamp)
		for message_type in found_message_types:
			if found_message_types[message_type] == False:
				rospy.logerr("Found %s param but no handler is provided. Ignoring.",message_type)
	
	def close_bag(self):
		self.bag.close()				

def get_args():
	try:
		opts, args = getopt.getopt(sys.argv[1:],"hi:o:",["ifile=","ofile="])
	except getopt.GetoptError:
		rospy.logerr(help_str())
		rospy.logerr("Job's interrupted. Exiting...")
		sys.exit()
	return opts, args

def handle_args(opts, args):
	functionfiles = []
	for opt, arg in opts:
		if opt in ('-h', "--help"):
			rospy.loginfo(help_str())
			sys.exit()
		elif opt in ("-i", "--ifile"):
			inputfile = arg
		elif opt in ("-o", "--ofile"):
			outputfile = arg
	for arg in args:
		functionfiles.append(arg)
	return inputfile, outputfile, functionfiles

if __name__ == '__main__':
	rospy.init_node('carmen2rosbag', anonymous=True)
	converter = bag_creator()
	if len(sys.argv) < 3:
		rospy.logerr(help_str())
		rospy.logerr("Job's interrupted. Exiting...")
		sys.exit()
	opts, args = get_args()
	inputfile, outputfile, functionfiles = handle_args(opts, args)
	
	converter.convert(inputfile, outputfile, functionfiles)
	converter.close_bag()
	rospy.loginfo("Job's done.")
