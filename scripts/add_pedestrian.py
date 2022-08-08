#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from smit_matlab_sim.srv import AddPedestrian

if __name__ == '__main__':
	rospy.wait_for_service('add_pedestrian')
	client = rospy.ServiceProxy('add_pedestrian', AddPedestrian)
	v = 0
	p = Float64MultiArray()
	p.data = [1, 1, 14, 14]
	p.layout = MultiArrayLayout()
	p.layout.dim = []
	p.layout.dim.append(MultiArrayDimension())
	p.layout.dim.append(MultiArrayDimension())
	p.layout.dim[0].label = "points"
	p.layout.dim[0].size = int(len(p.data)/2)
	p.layout.dim[0].stride = len(p.data)
	p.layout.dim[1].label = "coordinates"
	p.layout.dim[1].size = 2
	p.layout.dim[1].stride = 2
	p.layout.data_offset = 0
	print(client(v, p, True, True))

