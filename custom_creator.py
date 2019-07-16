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

ros_node = False

def help_str():
	common_str = "custom_creator.py -i path/to/input.log -o path/to/write/rosbag.bag [names_of_function_files_in_same_dir]"
	if ros_node:
		return "usage: rosrun carmen_publisher "+common_str
	else:
		return "./"+common_str

def print_info(var):
	if ros_node:
		rospy.loginfo(var)
	else:
		print(var)

def print_err(var):
	if ros_node:
		rospy.logerr(var)
	else:
		print(var)

def is_param_tag(string):
	return string=='PARAM'

class bag_creator:
	def __init__(self):
		self.modules = []
		self.params = {}

	def set_input_file(self, inputfile):
		try:
			inCompleteName = os.path.expanduser(inputfile)
			self.f = open(inCompleteName, "r")
			print_info("Reading data from: "+inCompleteName)
		except (IOError, ValueError):
			print_err("Couldn't open "+inputfile)
			print_err(help_str())
			print_err("Job's interrupted. Exiting...")
			exit(-1)
		return self.f
	
	def set_output_file(self, outputfile):
		try:
			outCompleteName = os.path.expanduser(outputfile)
			self.bag = rosbag.Bag(outCompleteName, "w")
			print_info("Writing rosbag to: "+outCompleteName)
		except (IOError, ValueError):
			print_err("Couldn't open "+outputfile)
			print_err(help_str())
			print_err("Job's interrupted. Exiting...")
			exit(-1)
		return self.bag
		
	def set_function_files(self, functionfiles):
		for functionfile in functionfiles:
			try:
				self.modules.append(__import__(functionfile.replace('.py', '')))
			except (ImportError, ValueError):
				print_err("Couldn't import "+functionfile)
				print_err(help_str())
				print_err("Job's interrupted. Exiting...")
				exit(-1)
		return self.modules
	
	def set_self_params(self, words):
		if words[0] in self.params:
			print_err("param "+words[1]+" is already set to")
			print_err(self.params[words[1]])
			print_err("rewriting to")
			print_err(words[2:])
		self.params[words[1]] = words[2:]
	
	def convert(self, inputfile, outputfile, functionfiles):
		self.set_input_file(inputfile)
		self.set_output_file(outputfile)
		self.set_function_files(functionfiles)
		print_info("Start convertion to rosbag")
		founded_msgs = {}
		for line in self.f:
			words = line.split()
			if words[0][0] == '#':
				continue
			message_type = words[0]
			if is_param_tag(message_type):
				self.set_self_params(words)
				continue
			found_message_type = False
			
			for module in self.modules:
				if message_type in module.message_types:
					found_message_type = True
					tupple_topic_message_time = module.create_messages(self.params, words)
					for topic, message, stamp in tupple_topic_message_time:
						self.bag.write(topic, message, stamp)
			if not found_message_type and message_type not in founded_msgs:
				print_info("Found "+message_type+" param but no handler is provided. Ignoring.")
			founded_msgs[message_type] = None
				
	
	def close_bag(self):
		self.bag.close()				

def get_args():
	try:
		opts, args = getopt.getopt(sys.argv[1:],"hi:o:",["ifile=","ofile="])
	except getopt.GetoptError:
		print_err(help_str())
		print_err("Job's interrupted. Exiting...")
		sys.exit()
	return opts, args

def handle_args(opts, args):
	functionfiles = []
	for opt, arg in opts:
		if opt in ('-h', "--help"):
			print_info(help_str())
			sys.exit()
		elif opt in ("-i", "--ifile"):
			inputfile = arg
		elif opt in ("-o", "--ofile"):
			outputfile = arg
	for arg in args:
		functionfiles.append(arg)
	return inputfile, outputfile, functionfiles

if __name__ == '__main__':
	if ros_node:
		rospy.init_node('carmen2rosbag', anonymous=True)
	converter = bag_creator()
	if len(sys.argv) < 3:
		print_err(help_str())
		print_err("Job's interrupted. Exiting...")
		sys.exit()
	opts, args = get_args()
	inputfile, outputfile, functionfiles = handle_args(opts, args)
	
	converter.convert(inputfile, outputfile, functionfiles)
	converter.close_bag()
	print_info("Job's done.")
