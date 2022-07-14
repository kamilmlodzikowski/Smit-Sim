#!/usr/bin/env python3
import numpy as np
import random
import argparse
import matplotlib.pyplot as plt
from PIL import Image
from datetime import datetime
from roboticstoolbox import DstarPlanner

class RandomMapServerWithPedestrians(object):
	"""docstring for RandomMapServerWithPedestrians"""
	def __init__(self, args):
		self.w = args.width
		self.h = args.height
		self.res = args.resolution

		self.wall_w = args.wall_width
		self.ext_wall = args.external_wall
		self.min_room_dim = args.min_room_dim
		self.door_w = args.door_width
		self.door_to_wall_min = args.door_to_wall_min
		self.max_depth = args.max_depth
		
		self.map = np.empty((self.h*self.res, self.w*self.res))

	def regenerate_map(self):
		# map has an external wall
		if self.ext_wall:
			self.map = np.ones((self.h*self.res, self.w*self.res))
			self.map[self.wall_w:-self.wall_w, self.wall_w:-self.wall_w] = 0
			self.add_wall(self.wall_w, self.h*self.res-self.wall_w, self.wall_w, self.w*self.res-self.wall_w)
		# map has no external wall
		else:
			self.map = np.zeros((self.h*self.res, self.w*self.res))
			self.add_wall(0, self.h*self.res, 0, self.w*self.res)

	def add_wall(self, hmin, hmax, wmin, wmax, depth = 1):
		# plot the current map state
		# if depth > 1:
		# 	self.plot()

		# maximum depth reached
		if depth > self.max_depth:
			print('Max depth reached')
			return

		# room is too small to add wall
		if hmax - hmin < 2*self.min_room_dim + self.wall_w and wmax - wmin < 2*self.min_room_dim + self.wall_w:
			print('Room too small to divide')
			return

		# divide room with wall across bigger dimension (horizontally if botyh axis are equal)
		if (hmax-hmin) >= (wmax-wmin):
			self.add_wall_horizontal(hmin, hmax, wmin, wmax, depth)
		else:
			self.add_wall_vertical(hmin, hmax, wmin, wmax, depth)

	def add_wall_horizontal(self, hmin, hmax, wmin, wmax, depth, retry = 0):
		# cancel the wall after too many retries
		if retry == 10:
			print('Max number of retries reached, wall aborted.')
			return

		# create wall
		wall_pos = random.randint(hmin + self.min_room_dim, hmax - self.min_room_dim - self.wall_w)
		print('Horizontal Wall: ' + str(wall_pos))
		# check for doors on the sides
		if depth > 1 and sum(self.map[wall_pos:(wall_pos + self.wall_w), wmin - 1]) < self.wall_w or sum(self.map[wall_pos:(wall_pos + self.wall_w), wmax]) < self.wall_w:
			print('Wall blocking the door on try ' + str(retry))
			self.add_wall_horizontal(hmin, hmax, wmin, wmax, depth, retry + 1)
			return
		self.map[wall_pos:(wall_pos + self.wall_w), wmin:wmax] = 1

		# create door
		door_pos = random.randint(wmin + self.door_to_wall_min, wmax - self.door_to_wall_min - self.door_w)
		print('Door: ' + str(door_pos))
		self.map[wall_pos:(wall_pos + self.wall_w), door_pos:(door_pos + self.door_w)] = 0

		# further split created rooms
		self.add_wall(hmin, wall_pos, wmin, wmax, depth + 1)
		self.add_wall(wall_pos + self.wall_w, hmax, wmin, wmax, depth + 1)

	def add_wall_vertical(self, hmin, hmax, wmin, wmax, depth, retry = 0):
		# cancel the wall after too many retries
		if retry == 10:
			print('Max number of retries reached, wall aborted.')
			return

		# create wall
		wall_pos = random.randint(wmin + self.min_room_dim, wmax - self.min_room_dim - self.wall_w)
		print('Vertical Wall: ' + str(wall_pos))
		# check for doors on the sides
		if depth > 1 and sum(self.map[hmin - 1, wall_pos:(wall_pos + self.wall_w)]) < self.wall_w or sum(self.map[hmax, wall_pos:(wall_pos + self.wall_w)]) < self.wall_w:
			print('Wall blocking the door on try ' + str(retry))
			self.add_wall_vertical(hmin, hmax, wmin, wmax, depth, retry + 1)
			return
		self.map[hmin:hmax, wall_pos:(wall_pos + self.wall_w)] = 1

		# create door
		door_pos = random.randint(hmin + self.door_to_wall_min, hmax - self.door_to_wall_min - self.door_w)
		print('Door: ' + str(door_pos))
		self.map[door_pos:(door_pos + self.door_w), wall_pos:(wall_pos + self.wall_w)] = 0

		# further split created rooms
		self.add_wall(hmin, hmax, wmin, wall_pos, depth + 1)
		self.add_wall(hmin, hmax, wall_pos + self.wall_w, wmax, depth + 1)



	def plot(self):
		rows, cols = np.shape(self.map)
		plt.figure()
		for row in range(rows):
			for col in range(cols):
				if self.map[row, col]:
					plt.plot(col, -row, color = 'black', marker = 's', markersize = 1)
		plt.show()

	def save_map_to_file(self, mapname, add_timestamp = False):
		filename = mapname + ('' if not add_timestamp else '_' + '_'.join(str(datetime.now().timestamp()).split('.')))

		# save map as image
		im = Image.fromarray(np.uint8(255 - self.map*255), 'L')
		im.save(filename + '.pgm')

		# save map parameters to file
		with open(filename + '.yaml', 'w') as f:
			f.write('image: ' + filename + '.pgm\nresolution: ' + str(1/self.res) + '\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')

	def __str__(self):
		return str(self.map)

if __name__ == '__main__':

    parser = argparse.ArgumentParser()

    # map metadata
    parser.add_argument("--width", type = int, default = 5)
    parser.add_argument("--height", type = int, default = 5)
    parser.add_argument("--resolution", type = int, default = 10)

    # map creation arguments
    parser.add_argument("--wall_width", type = int, default = 3)
    parser.add_argument("--external_wall", type = bool, default = True)
    parser.add_argument("--min_room_dim", type = int, default = 10)
    parser.add_argument("--door_width", type = int, default = 5)
    parser.add_argument("--door_to_wall_min", type = int, default = 2)
    parser.add_argument("--max_depth", type = int, default = 3)

    # pedestrian creation arguments
    parser.add_argument("--num_of_predestrians", type = int, default = 2)

    args = parser.parse_args()

    m = RandomMapServerWithPedestrians(args)
    m.regenerate_map()
    m.save_map_to_file('random_map', False)
    m.plot()

    planner = DstarPlanner(m.map, goal = (46, 46))
    planner.plan()
    path = planner.query(start = (4, 4))
    ds.plot(path)
    # planner.plot()
