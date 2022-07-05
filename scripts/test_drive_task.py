#!/usr/bin/env python3
from task import Drive
import numpy as np

if __name__ == '__main__':
	# Drive.globalPose = np.array([2, 2])
	t1=Drive([2, 2], [3, 3], 0.1)
	print('T1 Pose: ' + str(t1.pos))
	print('T1 path: ' + str(t1.path.points))
	print('T1 full distance: ' + str(t1.path.get_distance()))
	t1.do_work(2)
	print('T1 Work: 2')
	print('T1 Pose: ' + str(t1.pos))
	print('T1 path: ' + str(t1.path.points))
	print('T1 full distance: ' + str(t1.path.get_distance()))
	t1.do_work(20)
	print('T1 Work: 20')
	print('T1 Pose: ' + str(t1.pos))
	print('T1 path: ' + str(t1.path.points))
	print('T1 full distance: ' + str(t1.path.get_distance()))
	# t2 = Drive([2, 2], [2.5, 2.5], 0.01)
	# print('T2 Pose: ' + str(t2.pos))
	# print('T2 path: ' + str(t2.path.points))
	# print('T2 full distance: ' + str(t2.path.get_distance()))