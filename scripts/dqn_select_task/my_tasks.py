import numpy as np
import random
import sys
sys.path.insert(0, '../../../dqn')
from tasks import Task
from linear_path_ROS_planner import ROSNavigation

# Drive z punkt A do B
class Drive(Task):
	navigator = ROSNavigation()
	def __init__(self, pt1, pt2, spd):
		super().__init__()
		self.pos = np.array(pt1)
		self.goal = np.array(pt2)
		self.spd = spd
		self.path = Drive.navigator.plan(self.pos, self.goal)

	def dist(self, pos):
		return np.linalg.norm(pos - self.pos)

	def updatePos(self, pos):
		return self.pos

	def do_estimate(self):
		return self.path.get_distance()

	def do_wait(self, dt):
		pass

	def do_work(self, dt):
		self.path.step(self.spd, dt)
		self.pos = self.path.pos

	def is_alive(self):
		return True

	def __str__(self):
		dst = self.do_estimate()
		return f'D | dst: {dst:.2f} m | spd: {self.spd:.2f} m/s'

	__repr__ = __str__
	
	def serialize(self):
	  return f"{self.goal[0]}|{self.goal[1]}"


def DriveGenerator():
	x_min = 1
	x_max = 50
	y_min = 1
	y_max = 30
	x1 = x_min + random.random() * (x_max - x_min)
	x2 = x_min + random.random() * (x_max - x_min)
	y1 = y_min + random.random() * (y_max - y_min)
	y2 = y_min + random.random() * (y_max - y_min)
	spd_min = 0.1
	spd_max = 1.0
	spd = spd_min + random.random() * (spd_max - spd_min)
	return Drive([x1, y1], [x2, y2], spd)