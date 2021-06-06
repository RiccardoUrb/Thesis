#!/usr/bin/env python

import rospy
import struct
import ctypes
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String, Header


def callback(ros_point_cloud,):
	global iteration 
	# rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
	gen = pc2.read_points(ros_point_cloud, skip_nans=True)
	int_data = list(gen)
	xyz = np.array([[0, 0, 0]])
	# rgb = np.array([[0, 0, 0]])
	iteration += 1
	
	for x in int_data:
		xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
		# test = x[3] 
		# cast float32 to int so that bitwise operations are possible
		# s = struct.pack('>f' ,test)
		# i = struct.unpack('>l',s)[0]
		# you can get back the float value by the inverse operations
		# pack = ctypes.c_uint32(i).value
		# r = (pack & 0x00FF0000)>> 16
		# g = (pack & 0x0000FF00)>> 8
		# b = (pack & 0x000000FF)
		# prints r,g,b values in the 0-255 range
		# x,y,z can be retrieved from the x[0],x[1],x[2]
		# rgb = np.append(rgb,[[r,g,b]], axis = 0)
	xyz = np.unique(xyz, axis = 0)
	np.savetxt('listen' + str(iteration) + '.txt', xyz, fmt='%.3f')
	print('iteration done')



def listener():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber('/d435/depth/pcd', PointCloud2, callback, queue_size=1, buff_size=52428800)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
    iteration = 0
    listener()
