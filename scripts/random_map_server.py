#!/usr/bin/env python3
import numpy as np
import random
import argparse
import math
from statistics import mean
import matplotlib.pyplot as plt
from PIL import Image
from datetime import datetime
from enum import IntEnum
from roboticstoolbox import DistanceTransformPlanner, PRMPlanner
from linear_path import LinearPath

import rospy
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import OccupancyGrid
from smit_matlab_sim.srv import Step, AddPedestrian, AddPedestrianResponse, GetRoomsAndDoors, GetRoomsAndDoorsResponse
from smit_matlab_sim.msg import Room
from std_srvs.srv import Empty

class RandomMapServerNode(object):
	"""docstring for RandomMapServerNode"""
	def __init__(self, args):
		self.rms = RandomMapServerWithPedestrians(args)
		self.pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
		self.srv_step = rospy.Service('perform_pedestrians_step', Step, self.perform_step)
		self.srv_regenerate_map = rospy.Service('regenerate_map', Empty, self.regenerate_map)
		self.srv_regenerate_ped = rospy.Service('regenerate_pedestrians', Empty, self.regenerate_pedestrians)
		self.srv_add_ped = rospy.Service('add_pedestrian', AddPedestrian, self.add_pedestrian)
		self.srv_get_structures = rospy.Service('get_rooms_and_doors', GetRoomsAndDoors, self.get_structures)

		self.publish = args.publish
		self.rate = args.publish_rate
		self.auto_step = args.auto_step
		self.pub_on_step = args.publish_on_step

		self.msg = OccupancyGrid()
		self.msg.header.frame_id = "map"
		self.msg.info.width = self.rms.w
		self.msg.info.height = self.rms.h
		self.msg.info.resolution = self.rms.res
		self.msg.info.map_load_time = rospy.Time.now()
		self.msg.info.origin.position.x = 0
		self.msg.info.origin.position.y = 0
		self.msg.info.origin.position.z = 0
		self.msg.info.origin.orientation.x = 0
		self.msg.info.origin.orientation.y = 0
		self.msg.info.origin.orientation.z = 0
		self.msg.info.origin.orientation.w = 1

		if self.publish:
			self.timer = rospy.Timer(rospy.Duration(1.0/self.rate), self.publish_map)

	def perform_step(self, req):
		self.rms.step(req.time)
		if self.pub_on_step:
			self.publish_map()

	def add_pedestrian(self, req):
		if req.path.data:
			return AddPedestrianResponse(self.rms.add_pedestrian(req.velocity, np.array(path), req.full_path, req.behaviour))
		if len(req.path.layout.dim) == 2 and req.path.layout.dim[1].size == 2:
			path = []
			for i in range(req.path.layout.dim[0].size):
				path.append([req.path.data[req.path.layout.data_offset + req.path.layout.dim[1].stride*i], req.path.data[req.path.layout.data_offset + req.path.layout.dim[1].stride*i + 1]])
			return AddPedestrianResponse(self.rms.add_pedestrian(req.velocity, np.array(path), req.full_path, req.behaviour))
		else:
			return AddPedestrianResponse(False)


	def publish_map(self, event = None):
		self.msg.data = np.uint8(self.rms.get_pedmap().reshape(-1)*100)
		self.msg.header.stamp = rospy.Time.now()
		self.pub.publish(self.msg)
		if (self.auto_step):
			self.rms.step(1.0/self.rate)

	def regenerate_map(self, req):
		self.rms.regenerate_map()

	def regenerate_pedestrians(self, req):
		self.rms.regenerate_pedestrians()

	def get_structures(self, req):
		return GetRoomsAndDoorsResponse([Room(r["id"], [p*self.rms.res for p in r["x"]], [p*self.rms.res for p in r["y"]], r["doors"], r["neighbours"]) for r in self.rms.rooms], [Room(d["id"], [p*self.rms.res for p in d["x"]], [p*self.rms.res for p in d["y"]], [], []) for d in self.rms.doors], Room(self.rms.ext_door["id"], [p*self.rms.res for p in self.rms.ext_door["x"]], [p*self.rms.res for p in self.rms.ext_door["y"]], [], []) if self.rms.ext_ent else Room())

class PedestrianBehaviour(IntEnum):
	CIRCLE = 1
	RANDOM = 2
	DISSAPEAR = 3
	REROUTE = 4

class RandomMapServerWithPedestrians(object):

	"""docstring for RandomMapServerWithPedestrians"""
	def __init__(self, args):
		self.res = args.resolution
		self.w = round(args.width/self.res)
		self.h = round(args.height/self.res)

		self.wall_w = round(args.wall_width/self.res)
		self.ext_wall = args.external_wall
		self.ext_ent = args.external_entrance
		self.min_room_dim = round(args.min_room_dim/self.res)
		self.door_w = round(args.door_width/self.res)
		self.door_to_wall_min = round(args.door_to_wall_min/self.res)
		self.max_depth = args.max_depth
		
		self.map = np.empty((self.h, self.w))
		self.prob_map = np.zeros((self.h, self.w))
		self.norm_prob_map = np.zeros((self.h, self.w))
		self.scaled_prob_map = np.zeros((self.h, self.w))

		self.prob_room = args.room_probability
		self.prob_door = args.door_probability
		self.prob_ent = args.entrance_probability

		self.num_p = args.num_of_pedestrians
		self.p_min_sp = args.pedestrian_min_speed/self.res
		self.p_max_sp = args.pedestrian_max_speed/self.res
		self.p_rad = round(args.pedestrian_radius/self.res)
		self.foot_rad = round(args.pedestrian_foot_radius/self.res)
		self.p_def_beh = PedestrianBehaviour(args.pedestrian_behaviour)

		self.planner = PRMPlanner(self.map, distance = 'euclidean', inflate = self.p_rad + self.foot_rad, npoints = int((self.w*self.h)/100))
		if self.num_p > 0:
			# self.planner = DistanceTransformPlanner(self.map, distance = 'euclidean', inflate = self.p_rad + self.foot_rad)
			self.p = [LinearPath([0, 0], [[0, 0]]) for _ in range(self.num_p)]
			self.p_path = [[] for _ in range(self.num_p)]
			self.p_sp = [0 for _ in range(self.num_p)]
			self.p_beh = [self.p_def_beh for _ in range(self.num_p)]
		else:
			self.p = []
			self.p_path = []
			self.p_sp = []
			self.p_beh = []

		Y, X = np.ogrid[-self.foot_rad:self.foot_rad+1, -self.foot_rad:self.foot_rad+1]
		dist_from_center = np.sqrt((X)**2 + (Y)**2)
		self.foot_mask = dist_from_center <= self.foot_rad
		# print(self.foot_mask)

		self.rooms = []
		self.doors = []
		self.ext_door = {"id": 0, "x":[], "y":[]}
		self.regenerate_map()

	def regenerate_map(self):
		# map has an external wall
		if self.ext_wall:
			self.map = np.ones((self.h, self.w))
			self.map[self.wall_w:-self.wall_w, self.wall_w:-self.wall_w] = 0
			self.add_wall(self.wall_w, self.h-self.wall_w, self.wall_w, self.w-self.wall_w)
		# map has no external wall
		else:
			self.map = np.zeros((self.h, self.w))
			self.add_wall(0, self.h, 0, self.w)

		# set neighbours for rooms
		for r1 in self.rooms:
			for r2 in self.rooms[r1["id"]-1:]:
				if r1["id"] == r2["id"]:
					continue
				if list(set(r1["doors"]) & set(r2["doors"])):
					r1["neighbours"].append(r2["id"])
					r2["neighbours"].append(r1["id"])


		# create entrance door
		if self.ext_wall and self.ext_ent:
			self.add_external_door()

		# create probability maps
		self.regenerate_probability_map()

		# regenerate pedestrians if needed
		self.planner = PRMPlanner(self.map, distance = 'euclidean', inflate = self.p_rad + self.foot_rad, npoints = int((self.w*self.h)/100))
		self.planner.plan()
		if self.num_p > 0:
			self.regenerate_pedestrians()

	def regenerate_probability_map(self):
		for r in self.rooms:
			self.prob_map[r["y"][0]:r["y"][1], r["x"][0]:r["x"][1]] = self.prob_room
		for d in self.doors:
			self.prob_map[d["y"][0]:d["y"][1], d["x"][0]:d["x"][1]] = self.prob_door
		if self.ext_wall and self.ext_ent:
			d = self.ext_door
			self.prob_map[d["y"][0]:d["y"][1], d["x"][0]:d["x"][1]] = self.prob_ent
		max_p = self.prob_map.max()
		self.scaled_prob_map = self.prob_map/max_p
		sum_p = self.prob_map.sum()
		self.norm_prob_map = self.prob_map/sum_p

	def regenerate_pedestrians(self):
		for i in range(self.num_p):
			print("Pedestrian: " + str(i))
			while True:
				try:
					# start = (random.randrange(0, self.h), random.randrange(0, self.w))
					# goal = (random.randrange(0, self.h), random.randrange(0, self.w))
					start = self.get_random_point()
					goal = start
					while goal == start:
						goal = self.get_random_point()
					print(start)
					print(goal)
					self.p_path[i] = self.planner.query(start = start, goal = goal)
					self.p[i] = LinearPath(start, self.p_path[i][1:])
					self.p_sp[i] = random.uniform(self.p_min_sp, self.p_max_sp)
					break
				except ValueError as e:
					print(e)
					pass
				except RuntimeError as e:
					print(e)
					pass

	def add_wall(self, hmin, hmax, wmin, wmax, depth = 1, top_d = None, bot_d = None, left_d = None, right_d = None):
		# plot the current map state
		# if depth > 1:
		# 	self.plot()

		# maximum depth reached
		if depth > self.max_depth:
			print('Max depth reached')
			self.rooms.append({"id": len(self.rooms) + 1, "x": [wmin, wmax], "y": [hmin, hmax], 'neighbours': [], 'doors': list(filter(lambda d: d is not None, [top_d, bot_d, left_d, right_d]))})
			return

		# room is too small to add wall
		if hmax - hmin < 2*self.min_room_dim + self.wall_w and wmax - wmin < 2*self.min_room_dim + self.wall_w:
			print('Room too small to divide')
			self.rooms.append({"id": len(self.rooms) + 1, "x": [wmin, wmax], "y": [hmin, hmax], 'neighbours': [], 'doors': list(filter(lambda d: d is not None, [top_d, bot_d, left_d, right_d]))})
			return

		# divide room with wall across bigger dimension (horizontally if botyh axis are equal)
		if (hmax-hmin) >= (wmax-wmin):
			self.add_wall_horizontal(hmin, hmax, wmin, wmax, depth, 0, top_d, bot_d, left_d, right_d)
		else:
			self.add_wall_vertical(hmin, hmax, wmin, wmax, depth, 0, top_d, bot_d, left_d, right_d)

	def add_wall_horizontal(self, hmin, hmax, wmin, wmax, depth, retry = 0, top_d = None, bot_d = None, left_d = None, right_d = None):
		# cancel the wall after too many retries
		if retry == 10:
			print('Max number of retries reached, wall aborted.')
			self.rooms.append({"id": len(self.rooms) + 1, "x": [wmin, wmax], "y": [hmin, hmax], 'neighbours': [], 'doors': list(filter(lambda d: d is not None, [top_d, bot_d, left_d, right_d]))})
			return

		# create wall
		wall_pos = random.randint(hmin + self.min_room_dim, hmax - self.min_room_dim - self.wall_w)
		print('Horizontal Wall: ' + str(wall_pos))
		# check for doors on the sides
		if depth > 1 and sum(self.map[wall_pos:(wall_pos + self.wall_w), wmin - 1]) < self.wall_w or sum(self.map[wall_pos:(wall_pos + self.wall_w), wmax]) < self.wall_w:
			print('Wall blocking the door on try ' + str(retry))
			self.add_wall_horizontal(hmin, hmax, wmin, wmax, depth, retry + 1, top_d, bot_d, left_d, right_d)
			return
		self.map[wall_pos:(wall_pos + self.wall_w), wmin:wmax] = 1

		# create door
		door_pos = random.randint(wmin + self.door_to_wall_min, wmax - self.door_to_wall_min - self.door_w)
		print('Door: ' + str(door_pos))
		self.map[wall_pos:(wall_pos + self.wall_w), door_pos:(door_pos + self.door_w)] = 0
		d_id = - len(self.doors) - 1
		self.doors.append({"id": d_id, "y": [wall_pos, wall_pos + self.wall_w], "x": [door_pos, door_pos + self.door_w]})

		# decide where left and right doors go
		if left_d:
			if self.doors[-left_d-1]["y"][0] < wall_pos:
				bot_left_d = left_d
				top_left_d = None
			else:
				bot_left_d = None
				top_left_d = left_d
		else:
			bot_left_d = None
			top_left_d = None
		if right_d:
			if self.doors[-right_d-1]["y"][0] < wall_pos:
				bot_right_d = right_d
				top_right_d = None
			else:
				bot_right_d = None
				top_right_d = right_d
		else:
			bot_right_d = None
			top_right_d = None

		# further split created rooms
		self.add_wall(hmin, wall_pos, wmin, wmax, depth + 1, self.doors[- d_id - 1]["id"], bot_d, bot_left_d, bot_right_d)
		self.add_wall(wall_pos + self.wall_w, hmax, wmin, wmax, depth + 1, top_d, self.doors[- d_id - 1]["id"], top_left_d, top_right_d)

	def add_wall_vertical(self, hmin, hmax, wmin, wmax, depth, retry = 0, top_d = None, bot_d = None, left_d = None, right_d = None):
		# cancel the wall after too many retries
		if retry == 10:
			print('Max number of retries reached, wall aborted.')
			self.rooms.append({"id": len(self.rooms) + 1, "x": [wmin, wmax], "y": [hmin, hmax], 'neighbours': [], 'doors': list(filter(lambda d: d is not None, [top_d, bot_d, left_d, right_d]))})
			return

		# create wall
		wall_pos = random.randint(wmin + self.min_room_dim, wmax - self.min_room_dim - self.wall_w)
		print('Vertical Wall: ' + str(wall_pos))
		# check for doors on the sides
		if depth > 1 and sum(self.map[hmin - 1, wall_pos:(wall_pos + self.wall_w)]) < self.wall_w or sum(self.map[hmax, wall_pos:(wall_pos + self.wall_w)]) < self.wall_w:
			print('Wall blocking the door on try ' + str(retry))
			self.add_wall_vertical(hmin, hmax, wmin, wmax, depth, retry + 1, top_d, bot_d, left_d, right_d)
			return
		self.map[hmin:hmax, wall_pos:(wall_pos + self.wall_w)] = 1

		# create door
		door_pos = random.randint(hmin + self.door_to_wall_min, hmax - self.door_to_wall_min - self.door_w)
		print('Door: ' + str(door_pos))
		self.map[door_pos:(door_pos + self.door_w), wall_pos:(wall_pos + self.wall_w)] = 0
		d_id = - len(self.doors) - 1
		self.doors.append({"id": d_id, "x": [wall_pos, wall_pos + self.wall_w], "y": [door_pos, door_pos + self.door_w]})

		# decide where top and bottom doors go
		if bot_d:
			if self.doors[-bot_d-1]["x"][0] < wall_pos:
				left_bot_d = bot_d
				right_bot_d = None
			else:
				left_bot_d = None
				right_bot_d = bot_d
		else:
			left_bot_d = None
			right_bot_d = None
		if top_d:
			if self.doors[-top_d-1]["x"][0] < wall_pos:
				left_top_d = top_d
				right_top_d = None
			else:
				left_top_d = None
				right_top_d = top_d
		else:
			left_top_d = None
			right_top_d = None

		# further split created rooms
		self.add_wall(hmin, hmax, wmin, wall_pos, depth + 1, left_top_d, left_bot_d, left_d, self.doors[- d_id - 1]["id"])
		self.add_wall(hmin, hmax, wall_pos + self.wall_w, wmax, depth + 1, right_top_d, right_bot_d, self.doors[- d_id - 1]["id"], right_d)

	def add_external_door(self):
		if random.random() < 0.5: # horizontal
			door_pos = random.randint(self.wall_w, self.w - self.wall_w - self.door_w)
			self.ext_door["x"] = [door_pos, door_pos + self.door_w]
			if random.random() < 0.5: # bottom
				if sum(self.map[self.wall_w, door_pos:(door_pos + self.door_w)]) > 0:
					self.add_external_door()
					return
				self.map[0:self.wall_w, door_pos:(door_pos + self.door_w)] = 0
				self.ext_door["y"] = [0, self.wall_w]
				for r in self.rooms:
					if self.ext_door["x"][0] > r["x"][0] and self.ext_door["x"][0] < r["x"][1] and r["y"][0] == self.wall_w:
						r["doors"].append(0)
						break
			else: # top
				if sum(self.map[self.h - self.wall_w - 1, door_pos:(door_pos + self.door_w)]) > 0:
					self.add_external_door()
					return
				self.map[self.h - self.wall_w:self.h, door_pos:(door_pos + self.door_w)] = 0
				self.ext_door["y"] = [self.h - self.wall_w, self.h]
				for r in self.rooms:
					if self.ext_door["x"][0] > r["x"][0] and self.ext_door["x"][0] < r["x"][1] and r["y"][1] == self.h - self.wall_w:
						r["doors"].append(0)
						break
		else: # vertical
			door_pos = random.randint(self.wall_w, self.h - self.wall_w - self.door_w)
			self.ext_door["y"] = [door_pos, door_pos + self.door_w]
			if random.random() < 0.5: # left
				if sum(self.map[door_pos:(door_pos + self.door_w), self.wall_w]) > 0:
					self.add_external_door()
					return
				self.map[door_pos:(door_pos + self.door_w), 0:self.wall_w] = 0
				self.ext_door['x'] = [0, self.wall_w]
				for r in self.rooms:
					if self.ext_door["y"][0] > r["y"][0] and self.ext_door["y"][0] < r["y"][1] and r["x"][0] == self.wall_w:
						r["doors"].append(0)
						break
			else: # right
				if sum(self.map[door_pos:(door_pos + self.door_w), self.w - self.wall_w - 1]) > 0:
					self.add_external_door()
					return
				self.map[door_pos:(door_pos + self.door_w), self.w - self.wall_w:self.w] = 0
				self.ext_door["x"] = [self.w - self.wall_w , self.w]
				for r in self.rooms:
					if self.ext_door["y"][0] > r["y"][0] and self.ext_door["y"][0] < r["y"][1] and r["x"][1] == self.w - self.wall_w:
						r["doors"].append(0)
						break


	def get_random_point(self):
		point = np.random.choice(self.w*self.h, 1, p = self.norm_prob_map.reshape(-1))
		return [int(point % self.w), int(point/self.w)]

	def step(self, dt):
		if self.num_p > 0:
			to_pop = []
			for i in range(self.num_p):
				pos, time_left = self.p[i].step(self.p_sp[i], dt)
				if time_left:
					if self.p_beh[i] == PedestrianBehaviour.CIRCLE:
						while time_left:
							self.p_path[i] = np.flip(self.p_path[i], axis = 0)
							self.p[i] = LinearPath(self.p_path[i][0], self.p_path[i][1:])
							pos, time_left = self.p[i].step(self.p_sp[i], dt - time_left)
					elif self.p_beh[i] == PedestrianBehaviour.RANDOM:
						while time_left:
							try:
								start = self.get_random_point()
								goal = start
								while goal == start:
									goal = self.get_random_point()
								self.p_path[i] = self.planner.query(start = start, goal = goal)
								self.p[i] = LinearPath(start, self.p_path[i][1:])
								self.p_sp[i] = random.uniform(self.p_min_sp, self.p_max_sp)
								break
							except ValueError as e:
								print(e)
								pass
							except RuntimeError as e:
								print(e)
								pass
					elif self.p_beh[i] == PedestrianBehaviour.DISSAPEAR:
						to_pop.append(i)
					elif self.p_beh[i] == PedestrianBehaviour.REROUTE:
						while time_left:
							try:
								start = pos
								goal = self.get_random_point()
								self.p_path[i] = self.planner.query(start = start, goal = goal)
								self.p[i] = LinearPath(start, self.p_path[i][1:])
								# self.p_sp[i] = random.uniform(self.p_min_sp, self.p_max_sp)
								break
							except ValueError as e:
								print(e)
								pass
							except RuntimeError as e:
								print(e)
								pass
			if len(to_pop) > 0:
				to_pop.reverse()
				for i in to_pop:
					self.p.pop(i)
					self.p_sp.pop(i)
					self.p_path.pop(i)
					self.p_beh.pop(i)
					self.num_p -= 1

	def add_pedestrian(self, speed, path, planned, behaviour):
		# if self.num_p == 0:
		# 	self.planner = PRMPlanner(self.map, distance = 'euclidean', inflate = self.p_rad + self.foot_rad, npoints = int((self.w*self.h)/100))

		if path == []:
			while(True):
				try:
					start = self.get_random_point()
					goal = start
					while goal == start:
						goal = self.get_random_point()
					path = self.planner.query(start = start, goal = goal)
					break
				except ValueError as e:
					print(e)
					pass
				except RuntimeError as e:
					print(e)
					pass
		else:
			try:
				path = path*(1/self.res)
				start = (path[0][0], path[0][1])
				if not planned:
					goal = (path[1][0], path[1][1])
					path = self.planner.query(start = start, goal = goal)
			except ValueError as e:
				print(e)
				return False
			except RuntimeError as e:
				print(e)
				return False

		self.p_path.append(path)
		self.p.append(LinearPath(start, self.p_path[-1][1:]))
		if speed == 0:
			speed = random.uniform(self.p_min_sp, self.p_max_sp)
		else:
			speed = speed/self.res
		self.p_sp.append(speed)
		if behaviour == 0:
			behaviour = self.p_def_beh
		self.p_beh.append(PedestrianBehaviour(behaviour))

		self.num_p += 1

		return True


	def get_pedmap(self):
		m = np.zeros((self.h, self.w), dtype=np.bool)
		if self.num_p > 0:
			for p in self.p:
				dist = p.points[0] - p.pos
				angle = math.atan2(dist[1], dist[0])
				try:
					m[round(p.pos[1] + self.p_rad*math.sin(angle-math.pi/2))-self.foot_rad:round(p.pos[1] + self.p_rad*math.sin(angle-math.pi/2))+self.foot_rad+1,
						round(p.pos[0] + self.p_rad*math.cos(angle-math.pi/2))-self.foot_rad:round(p.pos[0] + self.p_rad*math.cos(angle-math.pi/2))+1+self.foot_rad] |= self.foot_mask
					m[round(p.pos[1] - self.p_rad*math.sin(angle-math.pi/2))-self.foot_rad:round(p.pos[1] - self.p_rad*math.sin(angle-math.pi/2))+self.foot_rad+1,
						round(p.pos[0] - self.p_rad*math.cos(angle-math.pi/2))-self.foot_rad:round(p.pos[0] - self.p_rad*math.cos(angle-math.pi/2))+1+self.foot_rad] |= self.foot_mask
				except ValueError:
					pass
		return np.maximum(m, self.map)


	def plot(self):
		rows, cols = np.shape(self.map)
		plt.figure()
		x = []
		y = []
		for row in range(rows):
			for col in range(cols):
				if self.map[row, col]:
					x.append(col)
					y.append(row)
		plt.scatter(x, y, color = 'black', marker = 's', s = 1)
		if self.num_p > 0:
			for i in range(self.num_p):
				if self.p_beh[i] == PedestrianBehaviour.CIRCLE:
					plt.plot(self.p_path[i].T[0], self.p_path[i].T[1], color = 'blue')
					plt.plot(self.p[i].pos[0], self.p[i].pos[1], color = 'blue', marker = 'o', markersize = 1)
					plt.text(self.p_path[i].T[0][0], self.p_path[i].T[1][0], str(i), color = 'blue')
					plt.text(self.p_path[i].T[0][-1], self.p_path[i].T[1][-1], str(i), color = 'blue')
		for r in self.rooms:
			plt.plot(r["x"], r["y"], color = "red")
			plt.text(mean(r["x"]), mean(r["y"]), str(r["id"]), color = 'red')
		for d in self.doors:
			plt.plot(d["x"], d["y"], color = "green")
			plt.text(mean(d["x"]), mean(d["y"]), str(d["id"]), color = 'green')
		if self.ext_wall and self.ext_ent:
			d =  self.ext_door
			plt.plot(d["x"], d["y"], color = "green")
			plt.text(mean(d["x"]), mean(d["y"]), str(d["id"]), color = 'green')
		plt.grid(b=None)
		plt.show()

	def plot_probability_map(self):
		rows, cols = np.shape(self.prob_map)
		plt.figure()
		ax = plt.axes()
		ax.set_facecolor("cyan")
		data = {}
		for row in range(rows):
			for col in range(cols):
				if self.prob_map[row, col]:
					color = str(1-self.scaled_prob_map[row, col])
					if color not in data.keys():
						data[color] = [[],[]]
					data[color][0].append(col)
					data[color][1].append(row)
		for color in data.keys():
			plt.scatter(data[color][0], data[color][1], color = color, marker = 's', s = 3)
		plt.grid(b=None)
		plt.show()

	def save_map_to_pgm(self, mapname, add_timestamp = False):
		filename = mapname + ('' if not add_timestamp else '_' + '_'.join(str(datetime.now().timestamp()).split('.')))

		# save map as image
		im = Image.fromarray(np.uint8(255 - np.flip(self.map, axis=0)*255), 'L')
		im.save(filename + '.pgm')

		# save map parameters to file
		with open(filename + '.yaml', 'w') as f:
			f.write('image: ' + filename + '.pgm\nresolution: ' + str(self.res) + '\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')

	def __str__(self):
		return str(self.map)

if __name__ == '__main__':

	parser = argparse.ArgumentParser()

	# map metadata
	parser.add_argument("--width", type = float, default = 15)
	parser.add_argument("--height", type = float, default = 15)
	parser.add_argument("--resolution", type = float, default = 0.1)

	# map creation arguments
	parser.add_argument("--wall_width", type = float, default = 0.3)
	parser.add_argument("--external_wall", type = bool, default = True)
	parser.add_argument("--external_entrance", type = bool, default = True)
	parser.add_argument("--min_room_dim", type = float, default = 3)
	parser.add_argument("--door_width", type = float, default = 1)
	parser.add_argument("--door_to_wall_min", type = float, default = 0.2)
	parser.add_argument("--max_depth", type = int, default = 4)

	# probability map arguments
	parser.add_argument("--room_probability", type = float, default = 10)
	parser.add_argument("--door_probability", type = float, default = 1)
	parser.add_argument("--entrance_probability", type = float, default = 10000)

	# pedestrian creation arguments
	parser.add_argument("--num_of_pedestrians", type = int, default = 5)
	parser.add_argument("--pedestrian_min_speed", type = float, default = 0.5)
	parser.add_argument("--pedestrian_max_speed", type = float, default = 2)
	parser.add_argument("--pedestrian_radius", type = float, default = 0.2)
	parser.add_argument("--pedestrian_foot_radius", type = float, default = 0.1)
	parser.add_argument("--pedestrian_behaviour", type = int, default = 1)

	# node arguments
	parser.add_argument("--publish", type = bool, default = True)
	parser.add_argument("--publish_rate", type = int, default = 100)
	parser.add_argument("--auto_step", type = bool, default = False)
	parser.add_argument("--publish_on_step", type = bool, default = False)

	args = parser.parse_args()

	# rms = RandomMapServerWithPedestrians(args)
	# rms.save_map_to_pgm('random_map', False)
	# rms.plot()
	# plt.savefig('random_map.jpg')

	rospy.init_node('random_map_test')
	node = RandomMapServerNode(args)
	for r in node.rms.rooms:
		print(r)
	node.rms.plot()
	node.rms.plot_probability_map()
	rospy.spin()
