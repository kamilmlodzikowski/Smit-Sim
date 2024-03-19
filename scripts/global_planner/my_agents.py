from TaskER.RequestTable import RequestTable, ScheduleRules, ScheduleRule, TaskerReqest

class RequestTableAgentPlugin:

	def __init__(self):
		self.reset()
	    self.rt = RequestTable()
		self.tasker_requests = []
		self.out = []
		self.profit = 0
		self.recently_updated = False

	def reset(self):
	    self.rt = RequestTable()
		self.tasker_requests = []
		self.out = []
		self.profit = 0

	def getTaskerRequestById(self, req_id):
		return [req for req in self.tasker_requests if req.id == req_id][0]

	def geJobById(self, jobs, job_id):
		return [job for job in jobs if job.id == job_id]

	def schedule(self, jobs, update_old = True):
		jobs_updated = False
		jobs_removed = False
		jobs_added = False
		for i,req in enumerate(self.tasker_requests):
			try:
				job = self.getJobById(jobs, req.id)
				if update_old:
					req.set_burst_time(job.estimated_duration)
			        req.evaluate_rules()
			        self.rt.updateRecord(req)
			        jobs_updated = True
			except IndexError:
			    self.rt.removeRecord_by_id(i)
				self.tasker_requests.pop(i)
				jobs.jobs_removed = True

		for job in jobs:
			try:
				req = self.getTaskerRequestById(job.id)
			except IndexError:
		        sr = ScheduleRules()
		        sr.addRule(ScheduleRule(rule_type='at', rule_value=t.getDeadline()))
		        req = TaskerReqest(ID=job.getID(),huid=job.getUUID(), plan_args='', req_time=job.calltime, shdl_rules=sr, priority=job.getPriority())
				req.set_burst_time(job.estimated_duration)
				req.evaluate_rules()
				self.rt.addRecord(req)
				self.tasker_requests.append(req)
				jobs_added = True

		self.recently_updated = jobs_added or jobs_removed or jobs_updated
		if self.recently_updated:
			self.out, self.profit = self.rt.schedule_with_priority()

class DecAgent:

	def __init__(self):
		self.selected_task = None
		self.plugins = []

	def select_task(jobs, now = None, eval_result = None)
		raise NotImplementedError()

class SchedulerAgent(DecAgent):

	def __init__(self):
		super(SchedulerAgent, self).__init__()
		self.plugins.append(RequestTableAgentPlugin())

	def select_task(self, jobs, now = None, eval_result = None):
		if not(self.selected_task is None):
			if not self.selected_task.preemptive:
				return self.selected_task

		self.plugins[0].schedule(jobs)

		if (not self.plugins[0].recently_updated) and not(self.selected_task is None):
			if self.selected_task.deadline >= now:
				return self.selected_task

		for sch in self.plugins[0].scheduled:
			if (sch.start - (sch.stop - sch.start)) <= now and sch.stop > self.now:
				for job in jobs:
					if job.id == sch.jobID:
						self.selected_task = job
						return self.selected_task



