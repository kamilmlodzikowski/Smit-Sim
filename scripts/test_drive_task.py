#!/usr/bin/env python
from tasks import Drive
import numpy as np

if __name__ == '__main__':
	Drive.globalPose = np.array([2, 2])
	t1=Drive([3, 3], 0.1)
	print('Pose: ' + str(Drive.globalPose))
	print('T1 path: ' + str(t1.path))
	print('T1 distances between points: '+  str(t1.distances))
	print('T1 full distance: ' + str(sum(t1.distances)))
	t1.do_work(2)
	print('Pose: ' + str(Drive.globalPose))
	print('T1 path: ' + str(t1.path))
	print('T1 distances between points: '+  str(t1.distances))
	print('T1 full distance: ' + str(sum(t1.distances)))
	t2 = Drive([2.5, 2.5], 0.01)
	print('Pose: ' + str(Drive.globalPose))
	print('T2 path: ' + str(t2.path))
	print('T2 distances between points: '+  str(t2.distances))
	print('T2 full distance: ' + str(sum(t2.distances)))