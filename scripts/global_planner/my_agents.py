from TaskER.RequestTable import RequestTable, ScheduleRules, ScheduleRule, TaskerReqest
import random

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

    def getJobById(self, jobs, job_id):
        return [job for job in jobs if job.id == job_id][0]

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
                self.rt.removeRecord_by_id(req.id)
                self.tasker_requests.pop(i)
                jobs_removed = True

        for job in jobs:
            try:
                req = self.getTaskerRequestById(job.id)
            except IndexError:
                sr = ScheduleRules()
                sr.addRule(ScheduleRule(rule_type='at', rule_value=job.getDeadline()))
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

    def select_task(jobs, now, eval_result):
        raise NotImplementedError()

class SchedulerAgent(DecAgent):

    def __init__(self):
        super(SchedulerAgent, self).__init__()
        self.plugins.append(RequestTableAgentPlugin())
        self.previous_job_count = 0
        self.previous_task = None
        self.previous_last_job_id = 0

    def select_task(self, jobs, now, eval_result = None):
        if len(jobs) == 0:
            self.selected_task = None
            return None

        if not(self.selected_task is None):
            if not self.selected_task.preemptive and self.selected_task in jobs:
                return self.selected_task

        if self.previous_job_count != len(jobs):
            self.plugins[0].schedule(jobs, update_old = False)
        elif self.previous_last_job_id != jobs[-1].id:
            self.plugins[0].schedule(jobs)

        self.previous_job_count = len(jobs)
        self.previous_last_job_id = jobs[-1].id

        if (not self.plugins[0].recently_updated) and not(self.selected_task is None):
            if self.selected_task.deadline >= now:
                return self.selected_task

        for sch in self.plugins[0].out.scheduled:
            if (sch.start - (sch.stop - sch.start)) <= now and sch.stop > now:
                for job in jobs:
                    if job.id == sch.jobID:
                        self.selected_task = job
                        return self.selected_task
                self.selected_task = None
                return None

class SimpleAgent(DecAgent):

    def __init__(self, hesitance = 0.0):
        super(SimpleAgent, self).__init__()
        self.hesitance = hesitance

    def select_task(self, jobs, now = None, eval_result = None):
        # print(f'Received jobs: {[job.id for job in jobs]}')
        if len(jobs) == 0:
            self.selected_task = None
            return None

        if not(self.selected_task is None):
            if (not self.selected_task.preemptive or random.random() <= self.hesitance) and self.selected_task in jobs:
                # print(self.selected_task in jobs)
                return self.selected_task

        if self.selected_task is None or not (self.selected_task in jobs):
            self.selected_task = jobs[0]

        for job in jobs:
            if job.estimated_duration < self.selected_task.estimated_duration:
                self.selected_task = job

        return self.selected_task

class DistanceAgent(DecAgent):

    def __init__(self, ratio = 0.0):
        super(DistanceAgent, self).__init__()
        self.ratio = ratio

    def select_task(self, jobs, now = None, eval_result = None):
        if len(jobs) == 0:
            self.selected_task = None
            return None

        if not(self.selected_task is None):
            if not self.selected_task.preemptive and self.selected_task in jobs:
                return self.selected_task

        if self.selected_task is None or not (self.selected_task in jobs):
            self.selected_task = jobs[0]

        selected_score = self.selected_task.distance_from_robot - self.ratio*self.selected_task.estimated_duration.seconds

        for job in jobs:
            score = job.distance_from_robot - self.ratio*job.estimated_duration.seconds
            if score > selected_score:
                self.selected_task = job
                selected_score = score

        return self.selected_task
