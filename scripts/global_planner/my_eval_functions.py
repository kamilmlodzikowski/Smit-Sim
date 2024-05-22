import datetime
import numpy as np
from my_tasks import Fall

class EvalResult:

	def __init__(self):
		self.terminate = False

class DQNEvalResult(EvalResult):

	def __init__(self):
		super(DQNEvalResult, self).__init__()
		self.reward = 0
		self.completed = False
		self.dead = False

class StatisticEvalResult(EvalResult):
	"""docstring for StatisticEvalResult"""
	def __init__(self):
		super(StatisticEvalResult, self).__init__()
		self.full_travel_distance = 0
		self.num_of_tasks_completed = []
		self.task_completion_to_deadline = []
		self.task_completion_to_deathtime = []
		self.num_of_tasks_interrupted = []
		self.task_interruptions = []

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
	def __init__(self, save_results = False):
		super(DQNEval, self).__init__(save_results)
		self.previous_action = None

		self.reward_real_job = 1
		self.reward_job_complete = 20
		self.reward_all_complete = 100

		self.penalty_nonexistent_job = 1
		self.penalty_change_job = 0.1
		self.penalty_dead_job = 200

	def calculate_results(self, tasks, current_job, now):
		reward = 0
		if current_job is None:
			reward = self.penalty_nonexistent_job
		else:
			if current_job.do_estimate() <= 0:
				reward = self.reward_job_complete
			else:
				reward = self.reward_real_job

		result = DQNEvalResult()
		result.completed = True

		for task in tasks:
			if task.do_estimate():
				result.completed = False
				break

		for task in tasks:
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

		if current_job != self.previous_action and not (self.previous_action is None):
			reward -= self.penalty_change_job

		result.reward = reward
		self.previous_action = current_job
		return result

	def reset(self):
		self.previous_action = None

class StatisticEval(EvalFunction):
	"""docstring for StatisticEval"""
	def __init__(self, system = None, task_types = [], dt = 5, recent_dt = 180, save_results = False):
		super(StatisticEval, self).__init__()

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

	def set_system(self, system):
		self.system = system
		self.last_robot_pos = self.system.pos
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
		self.recent_jobs = [None for i in range(int(self.recent_dt/self.dt) + 1)]
		self.last_robot_pos = np.array([0, 0]) if self.system is None else system.pos
		self.previous_humans_close_to_robot = []
		self.full_travel_distance = 0.0
		self.num_of_tasks_completed = [0 for _ in range(len(self.task_types))]
		self.task_completion_to_deadline = [] if self.system is None else [None for _ in range(len(self.system.tasks))]
		self.task_completion_to_deathtime = [] if self.system is None else [None for _ in range(len(self.system.tasks))]
		self.num_of_tasks_interrupted = [0 for _ in range(len(self.task_types))]
		self.task_interruptions = [] if self.system is None else [0 for _ in range(len(self.system.tasks))]
		self.num_of_human_abandonment = 0

	def calculate_results(self, tasks, current_job, now):
		result = StatisticEvalResult()

	    # przebyty dystans,
	    self.full_travel_distance += np.linalg.norm(system.pos - self.last_robot_pos)
	    result.full_travel_distance = self.full_travel_distance

		# wykonane poszczególne typy zadań
		self.num_of_tasks_completed = [0 for _ in range(len(self.task_types))]
		for task in tasks:
			if not task.do_estimate():
				for t, i in enumerate(self.task_types):
					if isinstance(task, t):
						self.num_of_tasks_completed[i] += 1
		result.num_of_tasks_completed = self.num_of_tasks_completed

	    # czas wykonania poszczególnych zadań względem czasu żądania (dla zadań, które zostały już wykonane)
	    for task, i in enumerate(tasks):
	    	if not task.do_estimate() and self.task_completion_to_deadline[i] is None:
	    		self.task_completion_to_deadline[i] = (now - task.deadline).seconds
	    result.task_completion_to_deadline = self.task_completion_to_deadline

	    # czas wykonania poszczególnych zadań upadku względem terminu (dla zadań, które zostały już wykonane)
	    for task, i in enumerate(tasks):
	    	if not task.do_estimate() and self.task_completion_to_deathtime[i] is None and isinstance(task.getDeathTime(), datetime.datetime):
	    		self.task_completion_to_deathtime[i] = (now - task.getDeathTime()).seconds
	    result.task_completion_to_deathtime = self.task_completion_to_deathtime

	    # liczbę przerwań każdego typu zadania
	    if self.previous_job != self.current_job and self.previous_job.do_estimate():
			for t, i in enumerate(self.task_types):
				if isinstance(self.previous_job, t):
					self.num_of_tasks_interrupted[i] += 1
	    result.num_of_tasks_interrupted = self.num_of_tasks_interrupted

	    # liczbę przerwań każdej instancji zadania
	    if self.previous_job != self.current_job and self.previous_job.do_estimate():
	    	for task, i in enumerate(tasks):
	    		if task == self.previous_job:
	    			self.task_interruptions[i] += 1
	    result.task_interruptions = self.task_interruptions

	    # Liczbę wyjść robota z okrągu o promieniu 2 m od miejsca upadku człowieka, kiedy robot wykonuje inne zadanie
	    close_to_human = []
	    for job in system.jobs:
	    	if isinstance(job, Fall) and job != current_job and np.linalg.norm(system.pos - job.pos) < 2:
	    		close_to_human.append(job)
	    for job in self.previous_humans_close_to_robot:
	    	if not (job in close_to_human) and job != current_job:
	    		self.num_of_human_abandonment += 1
	    result.num_of_human_abandonment = self.num_of_human_abandonment

	    # remember current state
	    self.previous_humans_close_to_robot = close_to_human
	    self.last_robot_pos = system.pos
	    self.previous_job = self.current_job

	    # Wykonania wszystkich zadań, lub
		result.terminate = True
		for task in tasks:
			if task.do_estimate():
				result.terminate = False
				break

	    # Wpadnięciu w drgania, zdefiniowane jako trzecie przełączenie na to samo zadanie w czasie mniejszym niż 3 min.
	    self.recent_jobs.append(current_job.uuid)
	    self.recent_jobs.pop(0)
	    if self.recent_jobs.count(self.recent_jobs[-1]) > 2:
	    	result.terminate = True
	    	return result

	    # Upłynięciu terminu któregoś zadania upadku
		for task in tasks:
			if not task.is_alive(now):
				result.terminate = True
				break

		return result

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


