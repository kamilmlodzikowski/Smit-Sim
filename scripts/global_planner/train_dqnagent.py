#!/usr/bin/env python3
import gym
from my_system import SystemConfig, System
from my_tasks import TaskConfig, TransportGenerator, FallGenerator, PickAndPlaceGenerator, Transport, Fall, PickAndPlace
from datetime import datetime, date, time
from my_agents import DQNAgent, DQNConfig
from my_eval_functions import DQNEval
from smit_sim.srv import FileOperation, FileOperationRequest
import rospy
import rospkg
import timeit
import json
import warnings
from rl.callbacks import Callback
import numpy as np
import os
from rl.policy import EpsGreedyQPolicy, LinearAnnealedPolicy
from rl.memory import SequentialMemory

class MyEpisodeLogger(Callback):
	def __init__(self, filepath):
		# Some algorithms compute multiple episodes at once since they are multi-threaded.
		# We therefore use a dictionary that is indexed by the episode to separate episodes
		# from each other.
		self.episode_start = {}
		self.observations = {}
		self.rewards = {}
		self.actions = {}
		self.metrics = {}
		self.step = 0
		self.data = []
		self.filepath = filepath

	def on_train_begin(self, logs):
		""" Print training values at beginning of training """
		self.train_start = timeit.default_timer()
		self.metrics_names = self.model.metrics_names
		print('Training for {} steps ...'.format(self.params['nb_steps']))

	def on_train_end(self, logs):
		""" Print training time at end of training """
		duration = timeit.default_timer() - self.train_start
		print('done, took {:.3f} seconds'.format(duration))
		arr_data = {k: [dic[k] for dic in self.data] for k in self.data[0]}
		with open(self.filepath, "w") as f:
			json.dump(arr_data, f)

	def on_episode_begin(self, episode, logs):
		""" Reset environment variables at beginning of each episode """
		self.episode_start[episode] = timeit.default_timer()
		self.observations[episode] = []
		self.rewards[episode] = []
		self.actions[episode] = []
		self.metrics[episode] = []

	def on_episode_end(self, episode, logs):
		""" Compute and print training statistics of the episode when done """
		duration = timeit.default_timer() - self.episode_start[episode]
		episode_steps = len(self.observations[episode])

		# Format all metrics.
		metrics = np.array(self.metrics[episode])
		metrics_template = ''
		metrics_variables = []
		with warnings.catch_warnings():
			warnings.filterwarnings('error')
			for idx, name in enumerate(self.metrics_names):
				if idx > 0:
					metrics_template += ', '
				try:
					value = np.nanmean(metrics[:, idx])
					metrics_template += '{}: {:f}'
				except Warning:
					value = '--'
					metrics_template += '{}: {}'
				metrics_variables += [name, value]
		
  
		variables = {
			'step': int(self.step),
			'nb_steps': int(self.params['nb_steps']),
			'episode': int(episode + 1),
			'duration': duration,
			'episode_steps': int(episode_steps),
			'sps': float(episode_steps) / duration,
			'episode_reward': np.sum(self.rewards[episode]),
			'reward_mean': np.mean(self.rewards[episode]),
			'reward_min': np.min(self.rewards[episode]),
			'reward_max': np.max(self.rewards[episode]),
			'action_mean': np.mean(self.actions[episode]),
			'action_min': float(np.min(self.actions[episode])),
			'action_max': float(np.max(self.actions[episode])),
			'obs_mean': np.mean(self.observations[episode]),
			'obs_min': np.min(self.observations[episode]),
			'obs_max': np.max(self.observations[episode]),
		}
		#print(template.format(**variables))
		self.data.append(variables)

		# Free up resources.
		del self.episode_start[episode]
		del self.observations[episode]
		del self.rewards[episode]
		del self.actions[episode]
		del self.metrics[episode]

	def on_step_end(self, step, logs):
		""" Update statistics of episode after each step """
		episode = logs['episode']
		self.observations[episode].append(logs['observation'])
		self.rewards[episode].append(logs['reward'])
		self.actions[episode].append(logs['action'])
		self.metrics[episode].append(logs['metrics'])
		self.step += 1

class DQNTrainingSystem(gym.Env):

	def __init__(self, sytem, agent, eval_function, tasks_per_type):
		rospy.init_node('smit_system')

		self.rospack = rospkg.RosPack()
		self.load_map_config_client = rospy.ServiceProxy('/load_config', FileOperation)
		self.system = system
		self.state_space = gym.spaces.Box(low=0, high=100000, shape=(len(self.system.task_config.task_desc), 5, 3))
		self.action_space = gym.spaces.Discrete(len(self.system.task_config.task_desc) * tasks_per_type)
		self.agent = agent
		self.eval_function = eval_function
		self.selected_task = None
		self.reset_count = 0

	def render(self):
		pass

	def close(self):
		pass

	def reset(self):
		# self.reset_count += 1
		# if self.reset_count > 800:
		# 	self.system.config.start = datetime.combine(date.today(), time(8, 0))
		# 	self.system.config.stop = datetime.combine(date.today(), time(12, 0))
		# 	self.system.task_config.count = 12
		# 	self.system.task_config.now = datetime.combine(date.today(), time(8, 0))
		# elif self.reset_count <= 400:
		# 	self.system.config.start = datetime.combine(date.today(), time(8, 0))
		# 	self.system.config.stop = datetime.combine(date.today(), time(8, 30))
		# 	self.system.task_config.count = 1
		# 	self.system.task_config.now = datetime.combine(date.today(), time(8, 0))
		# elif self.reset_count <= 800:
		# 	self.system.config.start = datetime.combine(date.today(), time(8, 0))
		# 	self.system.config.stop = datetime.combine(date.today(), time(9, 0))
		# 	self.system.task_config.count = 3
		# 	self.system.task_config.now = datetime.combine(date.today(), time(8, 0))
		self.load_map_config_client(FileOperationRequest('/'.join([self.rospack.get_path('smit_sim'), 'test_map'])))
		self.system.reset()
		self.agent.selected_task = None
		self.eval_function.system = system
		self.eval_function.reset()
		self.selected_task = None
		self.agent.calculate_state(self.system.jobs)
		while len(self.system.jobs) == 0:
			self.system.execute_step(self.selected_task)
			self.system.update_jobs()
		return self.agent.state

	def step(self, action):
		# print(f'[TIME] {self.system.now}')

		if not(self.selected_task is None):
			if (not self.selected_task.preemptive)  and self.selected_task in self.system.jobs:
				# print(f'[ACTION] Non-preemptive job is performed')
				pass
			else:
				self.selected_task = self.agent.tasks_in_state[action]
		else:
			self.selected_task = self.agent.tasks_in_state[action]

		# print(f'[ACTION] Received action {action}')
		# if not (self.selected_task is None):
			# print(f'[ACTION] Working on task {self.selected_task.uuid}, from available {self.agent.tasks_in_state}')

		self.system.execute_step(self.selected_task)
		self.system.update_jobs()

		result = self.eval_function.calculate_results(self.agent.tasks_in_state[action], system.now)
		reward = result.reward
		done = result.terminate
		status = "DEAD" if result.dead else ("DONE" if result.completed else ("TIME" if system.now >= system.config.stop else "WORK"))
		if status == "TIME":
			done = True

		# print(f'[JOBS] Passing jobs {self.system.jobs}')
		self.agent.calculate_state(self.system.jobs)
		state = agent.state
		return state, reward, done, {"status": status}


if __name__ == '__main__':
	rospy.wait_for_service('/load_config')
	sc = SystemConfig()
	sc.stop = datetime.combine(date.today(), time(10, 0))
	sc.use_estimator = False
	tc = TaskConfig([TransportGenerator, FallGenerator, PickAndPlaceGenerator], 5, sc.start, sc.stop - sc.start, seed = -1)
	tc.instant_call = True
	system = System(tc, sc)

	agent_config = DQNConfig()
	agent_config.training_steps = 5000000
	agent_config.policy = LinearAnnealedPolicy(EpsGreedyQPolicy(), 'eps', 1.0, 0.1, 0.0, 500000)
	agent_config.memory = SequentialMemory(limit=500000, window_length=1)
	agent_config.model_path = 'dqn_agent/' + datetime.now().strftime(f"%Y%m%d_%H%M%S_%f/")
	os.makedirs(agent_config.model_path)
	tasks_per_type = 5
	agent = DQNAgent(agent_config, [Transport, Fall, PickAndPlace], tasks_per_type)
	agent.trainig = True

	eval_func = DQNEval(system)

	gym = DQNTrainingSystem(system, agent, eval_func, tasks_per_type)


	agent.agent.fit(gym, nb_max_episode_steps=3000, nb_steps=agent_config.training_steps, visualize=False, verbose=2, callbacks = [MyEpisodeLogger(agent_config.model_path + "/history.json")])
	agent.save()
