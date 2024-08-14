import datetime
import numpy as np
from my_tasks import Fall, Task
import os
import csv

class EvalResult:

	def __init__(self):
		self.terminate = False

	def __str__(self):
		return f'Terminate: {self.terminate}'

class DQNEvalResult(EvalResult):

	def __init__(self):
		super(DQNEvalResult, self).__init__()
		self.reward = 0
		self.completed = False
		self.dead = False

	def __str__(self):
		return f'Terminate: {self.terminate}' + (f'result: {"completed" if self.completed else "dead"}' if self.terminate else '')

class StatisticEvalResult(EvalResult):
	"""docstring for StatisticEvalResult"""
	def __init__(self):
		super(StatisticEvalResult, self).__init__()
		self.completed = False
		self.dead = False
		self.oscillation = False
		self.full_travel_distance = 0
		self.num_of_tasks_completed = []
		self.task_completion_to_deadline = []
		self.task_completion_to_deathtime = []
		self.num_of_tasks_interrupted = []
		self.task_interruptions = []
		self.num_of_human_abandonment = 0

	def __str__(self):
		s = [f'Terminate: {self.terminate}' + (f" result: {'completed' if self.completed else ('dead' if self.dead else 'oscillation')}" if self. terminate else ""),
			f'Full travel distance:\n\t{self.full_travel_distance}',
			f'Number of tasks completed per type\n\t{self.num_of_tasks_completed}',
			f'Difference between task completion time and deadline\n\t{self.task_completion_to_deadline}',
			f'Difference between task completion time and deathtime\n\t{self.task_completion_to_deathtime}',
			f'Number of task interruptions per task type\n\t{self.num_of_tasks_interrupted}',
			f'Number of task interruptions per task\n\t{self.task_interruptions}',
			f'Number of humans abandonned while doing different tasks\n\t{self.num_of_human_abandonment}',
		]
		return '\n'.join(s)


class EvalFunction:
	"""docstring for EvalFunction"""
	def __init__(self, save_results = False):
		self.save = save_results
		
	def evaluate(self, tasks, current_job, now):
		eval_result = self.calculate_results(tasks, current_job, now)
		self.save_results()
		return eval_result

	def calculate_results(self, tasks, current_job, now):
		raise NotImplementedError()

	def save_results(self, eval_result):
		pass

	def reset(self):
		pass

class DQNEval(EvalFunction):
	"""docstring for DQNEval"""
	def __init__(self, system, save_results = False):
		super(DQNEval, self).__init__(save_results)
		self.previous_action = None

		self.reward_real_job = 0.2
		self.reward_job_complete = 0.5
		self.reward_all_complete = 1

		self.penalty_nonexistent_job = 0.5
		self.penalty_change_job = 0.05
		self.penalty_dead_job = 1

		self.system = system

	def calculate_results(self, current_job, now):
		reward = 0
		if current_job is None:
			if len(self.system.jobs):
				reward = -self.penalty_nonexistent_job
		else:
			if current_job.do_estimate() <= 0:
				reward = self.reward_job_complete
			else:
				reward = self.reward_real_job

		result = DQNEvalResult()
		result.completed = True

		for task in self.system.tasks:
			if task.do_estimate():
				result.completed = False
				break

		for task in self.system.tasks:
			if not task.is_alive(now):
				result.dead = True
				result.terminate = True
				result.completed = False
				reward = -self.penalty_dead_job
				result.reward = reward
				self.previous_action = current_job
				return result

		if result.completed:
			result.terminate = True
			reward = self.reward_all_complete

		elif current_job != self.previous_action and not (self.previous_action is None):
			if self.previous_action.do_estimate():
				reward -= self.penalty_change_job

		result.reward = reward
		self.previous_action = current_job
		return result

	def reset(self):
		self.previous_action = None

class StatisticEval(EvalFunction):
	"""docstring for StatisticEval"""
	def __init__(self, system = None, task_types = [], dt = 5, recent_dt = 180, save_results = False, save_file = 'statistical_eval.csv'):
		super(StatisticEval, self).__init__(save_results)

		self.save_filename = save_file
		self.save_file = None

		# adding system handle for better access
		self.system = system
		self.task_types = task_types
		self.last_robot_pos = np.array([0, 0]) if self.system is None else system.pos
		self.previous_job = None
		self.previous_humans_close_to_robot = []

		# recent job list for termination
		self.dt = dt
		self.recent_dt = recent_dt
		self.recent_jobs = [None for _ in range(int(self.recent_dt/self.dt) + 1)]

		# evaluation values
		self.full_travel_distance = 0.0
		self.num_of_tasks_completed = [0 for _ in range(len(self.task_types))]
		self.task_completion_to_deadline = [] if self.system is None else [None for _ in range(len(self.system.tasks))]
		self.task_completion_to_deathtime = [] if self.system is None else [None for _ in range(len(self.system.tasks))]
		self.num_of_tasks_interrupted = [0 for _ in range(len(self.task_types))]
		self.task_interruptions = [] if self.system is None else [0 for _ in range(len(self.system.tasks))]
		self.num_of_human_abandonment = 0

		self.reset()

	def set_system(self, system):
		self.system = system
		self.last_robot_pos = np.array([0, 0]) if self.system is None else self.system.pos
		self.task_completion_to_deadline = [] if self.system is None else [None for _ in range(len(self.system.tasks))]
		self.task_completion_to_deathtime = [] if self.system is None else [None for _ in range(len(self.system.tasks))]
		self.task_interruptions = [] if self.system is None else [0 for _ in range(len(self.system.tasks))]

	def set_dt(self, dt):
		self.dt = dt
		self.recent_jobs = [None for i in range(int(self.recent_dt/self.dt) + 1)]

	def set_recent_dt(self, dt):
		self.recent_dt = dt
		self.recent_jobs = [None for i in range(int(self.recent_dt/self.dt) + 1)]

	def set_task_types(self, task_types):
		self.task_types = task_types
		self.num_of_tasks_completed = [0 for _ in range(len(self.task_types))]
		self.num_of_tasks_interrupted = [0 for _ in range(len(self.task_types))]

	def reset(self):
		self.previous_job = None
		self.recent_jobs = [None for i in range(int(self.recent_dt/self.dt) + 1)]
		self.last_robot_pos = np.array([0, 0]) if self.system is None else self.system.pos
		self.previous_humans_close_to_robot = []
		self.full_travel_distance = 0.0
		self.num_of_tasks_completed = [0 for _ in range(len(self.task_types))]
		self.task_completion_to_deadline = [] if self.system is None else [None for _ in range(len(self.system.tasks))]
		self.task_completion_to_deathtime = [] if self.system is None else [None for _ in range(len(self.system.tasks))]
		self.num_of_tasks_interrupted = [0 for _ in range(len(self.task_types))]
		self.task_interruptions = [] if self.system is None else [0 for _ in range(len(self.system.tasks))]
		self.num_of_human_abandonment = 0

		if self.save:
			print(self.save_filename)
			if '/' in self.save_filename:
				try:
					os.makedirs('/'.join(self.save_filename.split('/')[:-1]))
				except Exception as e:
					print(e)
					pass
			with open(self.save_filename, 'a', newline='') as csvfile:
				csvwriter = csv.writer(csvfile, delimiter=';')
				csvwriter.writerow(['Full travel distance',
					'Number of tasks completed per type',
					'Diffetence between task completion time and deadline',
					'Diffetence between task completion time and deathtime',
					'Number of interruptions per task type',
					'Number of interruptions per task',
					'Number of times robot abandonned human',
					'Current job',
					'All tasks completed',
					'Some tasks are dead',
					'Agent fell into oscillation'
					])

	def calculate_results(self, tasks, current_job, now):
		result = StatisticEvalResult()

		# przebyty dystans,
		self.full_travel_distance += np.linalg.norm(self.system.pos - self.last_robot_pos)
		result.full_travel_distance = self.full_travel_distance

		# # wykonane poszczególne typy zadań
		# self.num_of_tasks_completed = [0 for _ in range(len(self.task_types))]
		# for task in tasks:
		# 	if not task.do_estimate():
		# 		for i, t in enumerate(self.task_types):
		# 			if isinstance(task, t):
		# 				self.num_of_tasks_completed[i] += 1
		# result.num_of_tasks_completed = self.num_of_tasks_completed

		# # czas wykonania poszczególnych zadań względem czasu żądania (dla zadań, które zostały już wykonane)
		# for i, task in enumerate(tasks):
		# 	if not task.do_estimate() and self.task_completion_to_deadline[i] is None:
		# 		self.task_completion_to_deadline[i] = (now - task.deadline).seconds
		# result.task_completion_to_deadline = self.task_completion_to_deadline

		# # czas wykonania poszczególnych zadań upadku względem terminu (dla zadań, które zostały już wykonane)
		# for i, task in enumerate(tasks):
		# 	if not task.do_estimate() and self.task_completion_to_deathtime[i] is None and isinstance(task.getDeathTime(), datetime.datetime):
		# 		self.task_completion_to_deathtime[i] = (now - task.getDeathTime()).seconds
		# result.task_completion_to_deathtime = self.task_completion_to_deathtime

		self.num_of_tasks_completed = [0 for _ in range(len(self.task_types))]
		for i, task in enumerate(tasks):
			if not task.do_estimate():
		# wykonane poszczególne typy zadań
				for j, t in enumerate(self.task_types):
					if isinstance(task, t):
						self.num_of_tasks_completed[j] += 1
		# czas wykonania poszczególnych zadań względem czasu żądania (dla zadań, które zostały już wykonane)
				if self.task_completion_to_deadline[i] is None:
					self.task_completion_to_deadline[i] = (1 if now >= task.deadline else -1)*abs(now - task.deadline).seconds
		# czas wykonania poszczególnych zadań upadku względem terminu (dla zadań, które zostały już wykonane)
				if self.task_completion_to_deathtime[i] is None and isinstance(task.getDeathTime(), datetime.datetime):
					self.task_completion_to_deathtime[i] = (1 if now >= task.getDeathTime() else -1)*abs(now - task.getDeathTime()).seconds
		# liczbę przerwań każdej instancji zadania
			if self.previous_job != current_job and (False if self.previous_job is None else self.previous_job.do_estimate()):
				if task == self.previous_job:
					self.task_interruptions[i] += 1
		result.num_of_tasks_completed = self.num_of_tasks_completed
		result.task_completion_to_deadline = self.task_completion_to_deadline
		result.task_completion_to_deathtime = self.task_completion_to_deathtime
		result.task_interruptions = self.task_interruptions


		# liczbę przerwań każdego typu zadania
		if self.previous_job != current_job and (False if self.previous_job is None else self.previous_job.do_estimate()):
			for i, t in enumerate(self.task_types):
				if isinstance(self.previous_job, t):
					self.num_of_tasks_interrupted[i] += 1
		result.num_of_tasks_interrupted = self.num_of_tasks_interrupted

		# # liczbę przerwań każdej instancji zadania
		# if self.previous_job != current_job and (False if self.previous_job is None else self.previous_job.do_estimate()):
		# 	for i, task in enumerate(tasks):
		# 		if task == self.previous_job:
		# 			self.task_interruptions[i] += 1
		# result.task_interruptions = self.task_interruptions

		# Liczbę wyjść robota z okrągu o promieniu 2 m od miejsca upadku człowieka, kiedy robot wykonuje inne zadanie
		close_to_human = []
		for job in self.system.jobs:
			if isinstance(job, Fall) and job != current_job and np.linalg.norm(self.system.pos - job.pos) < 2:
				close_to_human.append(job)
		for job in self.previous_humans_close_to_robot:
			if not (job in close_to_human) and job != current_job:
				self.num_of_human_abandonment += 1
		result.num_of_human_abandonment = self.num_of_human_abandonment

		# remember current state
		self.previous_humans_close_to_robot = close_to_human
		self.last_robot_pos = self.system.pos
		self.previous_job = current_job

		# Wykonania wszystkich zadań, lub
		result.terminate = True
		for task in tasks:
			if task.do_estimate():
				result.terminate = False
				break
		result.completed = result.terminate

		# Wpadnięciu w drgania, zdefiniowane jako trzecie przełączenie na to samo zadanie w czasie mniejszym niż 3 min.
		self.recent_jobs.append(None if current_job is None else current_job.uuid)
		self.recent_jobs.pop(0)
		if (not current_job is None) and self.recent_jobs.count(self.recent_jobs[-1]) > 2:
			indexes = [i for i,x in enumerate(self.recent_jobs) if x == self.recent_jobs[-1]]
			differences = [j-i for i, j in zip(indexes[:-1], indexes[1:])]
			if sum(i > 1 for i in differences) > 2:
				result.terminate = True
				result.oscillation = True

		# Upłynięciu terminu któregoś zadania upadku
		for task in tasks:
			if not task.is_alive(now):
				result.terminate = True
				result.dead = True
				break

		if self.save:
			self.save_results(result)
		return result

	def save_results(self, result):
		with open(self.save_filename, 'a', newline='') as csvfile:
			csvwriter = csv.writer(csvfile, delimiter=';')
			csvwriter.writerow([self.full_travel_distance,
				self.num_of_tasks_completed,
				self.task_completion_to_deadline,
				self.task_completion_to_deathtime,
				self.num_of_tasks_interrupted,
				self.task_interruptions,
				self.num_of_human_abandonment,
				self.previous_job.uuid if isinstance(self.previous_job, Task) else 'None',
				result.completed,
				result.dead,
				result.oscillation,
				])
		

# W naszym przykładzie zastosujmy funkcję ewaluacji, która na bieżąco liczy:

#     przebyty dystans,
#     wykonane poszczególne typy zadań
#     czas wykonania poszczególnych zadań względem czasu żądania (dla zadań, które zostały już wykonane)
#     czas wykonania poszczególnych zadań upadku względem terminu (dla zadań, które zostały już wykonane)
#     liczbę przerwań każdego typu zadania
#     liczbę przerwań każdej instancji zadania
#     Liczbę wyjść robota z okrągu o promieniu 2 m od miejsca upadku człowieka, kiedy robot wykonuje inne zadanie


# Funkcja wywołuje terminate() w momencie:

#     Wykonania wszystkich zadań, lub
#     Upłynięciu terminu któregoś zadania upadku
#     Wpadnięciu w drgania, zdefiniowane jako trzecie przełączenie na to samo zadanie w czasie mniejszym niż 3 min.


