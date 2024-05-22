class EvalResult:

	def __init__(self):
		self.terminate = False

class DQNEvalResult(EvalResult):

	def __init__(self):
		super(DQNEvalResult, self).__init__()
		self.reward = 0
		self.completed = False
		self.dead = False

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
