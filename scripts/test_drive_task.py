#!/usr/bin/env python
from tasks import Drive
import numpy as np

if __name__ == '__main__':
	Drive.globalPose = np.array([2, 2])
	t1=Drive([3, 3], 0.1)
	print(Drive.globalPose)
	print(t1.path)
	print(t1.distances)
	print(sum(t1.distances))
	t1.do_work(1)
	print(Drive.globalPose)
	print(t1.path)
	print(t1.distances)
	print(sum(t1.distances))
	t2 = Drive([2.5, 2.5], 0.01)
	print(Drive.globalPose)
	print(t2.path)
	print(t2.distances)
	print(sum(t2.distances))