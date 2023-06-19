import random
import numpy as np
import sys
import os
from shutil import copyfile
from datetime import datetime, timedelta, date, time
import math as m
sys.path.insert(0, '../')
from my_tasks import Transport
from linear_path_ROS_planner import ROSNavigation
sys.path.insert(0, '../../../tasker/src/TaskER/')
from RequestTable import RequestTable, ScheduleRules, ScheduleRule, TaskerReqest

class SystemConfig(object):
  def __init__(self):
    self.robot_speed = 0.5
    self.time_horizon = timedelta(hours = 1)
    self.start = datetime.combine(date.today(), time(8, 0))
    self.stop = datetime.combine(date.today(), time(20, 0))
    self.time_slot = timedelta(minutes = 5)
    self.dt = timedelta(seconds = 5)
    self.save = True
    self.prefix = ""
    self.day = 1

class System():
  def __init__(self, task_config, config = SystemConfig()):
    self.config = config
    self.task_config = task_config
    self.tasks = []
    self.now = self.config.start
    
    self.slot_num = int(self.config.time_horizon/self.config.time_slot)
    self.slot_num_full = int((self.config.stop - self.config.start)/self.config.time_slot)
    self.state_shape = (3 * self.slot_num)
    self.state = np.zeros(self.state_shape)
    self.navigator = ROSNavigation()
    self.reset()

  def render(self, mode="save"):
    print('')

  def close(self):
    if self.config.save:
      self.file_out.close()

  def reset(self):
    print('Resetting...')
    self.now = self.config.start
    self.tasks = self.task_config.generate()

    # initial agent position
    x_min = 1
    x_max = 9
    y_min = 1
    y_max = 9
    x1 = x_min + random.random() * (x_max - x_min)
    y1 = y_min + random.random() * (y_max - y_min)
    self.pos = np.array([x1, y1])

    # initialize tasker
    self.rt = RequestTable()
    self.jobs = []
    for i, t in enumerate(self.tasks):
      sr = ScheduleRules()
      sr.addRule(ScheduleRule(rule_type='at', rule_value=t.getDeadline()))
      job = TaskerReqest(ID=t.getID(),huid=t.getUUID(), plan_args='', req_time=self.now, shdl_rules=sr, priority=t.getPriority())
      job.set_burst_time(t.getBurst() + timedelta(seconds = self.navigator.plan(self.pos, t.pos).get_distance() / self.config.robot_speed))
      job.evaluate_rules()
      self.rt.addRecord(job)
      self.jobs.append(job)
    self.out, self.profit = self.rt.schedule_with_priority()

    # self.state = np.zeros(self.state_shape)
    # for s in range(self.slot_num):
    #   start = self.now + s*self.config.time_slot
    #   stop = self.now + (s+1)*self.config.time_slot
    #   for i,j in enumerate(self.jobs):
    #     if j.start_time >= start and j.start_time < stop or j.deadline > start and j.deadline <= stop or j.start_time < start and j.deadline > stop:
    #       self.state[s*3] += j.priority
    #       self.state[2+s*3] += 1
    #   for i,j in enumerate(self.out.scheduled):
    #     if j.start >= start and j.start < stop or j.stop > start and j.stop <= stop or j.start < start and j.stop > stop:
    #       self.state[1+s*3] += self.rt.get_request(j.jobID).priority

    print("Reset done")
  
    self.fname = ""
    if self.config.save:
      now = datetime.now() # current date and time
      fname = self.config.prefix + now.strftime(f"%Y%m%d_%H%M%S_%f_{self.config.day}")
      self.fname = 'timeseries/' + fname + '.csv'
      # copyfile('/tmp/tasker_chart_with_rejected.html', 'timeseries/' + fname + '_tcwr.html')
      # copyfile('/tmp/tasker_chart_no_rejected.html', 'timeseries/' + fname + '_tcnr.html')
      if not os.path.exists('/'.join(self.fname.split('/')[:-1])):
        os.makedirs('/'.join(self.fname.split('/')[:-1]))
      self.file_out = open(self.fname, "w")
      # TODO
      # self.file_out.write(str((self.now - self.config.start).seconds/(self.config.stop - self.config.start).seconds*self.slot_num_full/self.slot_num) + ':' + str(self.config.day) + ':' + str([s for s in self.state]) + '\n')
      for n,job in enumerate(self.jobs):
        self.file_out.write(':'.join([
          str((self.now - self.config.start).seconds/(self.config.stop - self.config.start).seconds*self.slot_num_full/self.slot_num),
          str(self.config.day),
          str((job.start_time - self.config.start).seconds/(self.config.stop - self.config.start).seconds*self.slot_num_full/self.slot_num),
          str(self.tasks[n].pos[0]),
          str(self.tasks[n].pos[1]),
          str(self.tasks[n].goal[0] if isinstance(self.tasks[n], Transport) else self.tasks[n].pos[0]),
          str(self.tasks[n].goal[1] if isinstance(self.tasks[n], Transport) else self.tasks[n].pos[1]),
          str(self.navigator.plan(self.pos, self.tasks[n].pos).get_distance()),
          str(job.priority),
          str(job.burst_time.seconds/(self.config.stop - self.config.start).seconds*self.slot_num_full/self.slot_num),
          '\n',
        ]))

  def step_until(self, deadline):
    if self.now >= deadline:
      print('Deadline reached')
      return

    self.dt = min(self.config.dt, deadline - self.now)

    for i,j in enumerate(self.out.scheduled):

      if j.start <= self.now + self.dt and j.stop > self.now:

        if j.start >= self.now:
          time_left = self.dt.seconds - (j.start - self.now).seconds
        else:
          time_left = self.dt.seconds

        # do the actual travel to the action spot if distanse from the agent is greater than threshold
        if self.tasks[int(j.jobID)].dist(self.pos) > 0.1:
          path = self.navigator.plan(self.pos, self.tasks[int(j.jobID)].pos)
          [self.pos, time_left] = path.step(self.config.robot_speed, time_left)

        # work on a task
        if time_left > 0:
          self.tasks[int(j.jobID)].do_work(time_left)
          self.pos = self.tasks[int(j.jobID)].pos

        # if task is done remove it from TaskER
        if not self.tasks[int(j.jobID)].do_estimate():
          self.rt.removeRecord_by_id(j.jobID)
          print('Job ' + j.jobID + ' complete')
          # update starting times for non-complete jobs depending on the robot's location
          for n,job in enumerate(self.jobs):
            task = self.tasks[n]
            if task.do_estimate() and job.start_time < self.now + self.config.time_slot and job.start_time > self.now - self.config.time_slot:
              burst = task.getBurst() + timedelta(seconds = self.navigator.plan(self.pos, task.pos).get_distance() / self.config.robot_speed)
              job.set_burst_time(burst)
              job.evaluate_rules()
              self.rt.updateRecord(job)
          self.out, self.profit = self.rt.schedule_with_priority()
        else:
          print('Worked on job ' + j.jobID + ': ' + str(self.tasks[int(j.jobID)].do_estimate()))
        break

    # continue work
    self.now = self.now + self.dt
    self.step_until(deadline)

  def run_env(self):
    while(self.now < self.config.stop):
      print("Stepping from " + str(self.now) + " to " + str(self.now + self.config.time_slot))
      self.step_until(self.now + self.config.time_slot)

      # self.state = np.zeros(self.state_shape)
      # for s in range(self.slot_num):
      #   start = self.now + s*self.config.time_slot
      #   stop = self.now + (s+1)*self.config.time_slot
      #   for i,j in enumerate(self.jobs):
      #     if j.start_time >= start and j.start_time < stop or j.deadline > start and j.deadline <= stop or j.start_time < start and j.deadline > stop:
      #       self.state[s*3] += j.priority
      #       self.state[2+s*3] += 1
      #   for i,j in enumerate(self.out.scheduled):
      #     if j.start >= start and j.start < stop or j.stop > start and j.stop <= stop or j.start < start and j.stop > stop:
      #       self.state[1+s*3] += self.rt.get_request(j.jobID).priority
            
      if self.config.save:
        # TODO
        # self.file_out.write(str((self.now - self.config.start).seconds/(self.config.stop - self.config.start).seconds*self.slot_num_full/self.slot_num) + ':' + str(self.config.day) + ':' + str([s for s in self.state]) + '\n')
        for n,task in enumerate(self.tasks):
          if self.tasks[n].do_estimate():
            self.file_out.write(':'.join([
              str((self.now - self.config.start).seconds/(self.config.stop - self.config.start).seconds*self.slot_num_full/self.slot_num),
              str(self.config.day),
              str((self.jobs[n].start_time - self.config.start).seconds/(self.config.stop - self.config.start).seconds*self.slot_num_full/self.slot_num),
              str(task.pos[0]),
              str(task.pos[1]),
              str(task.goal[0] if isinstance(self.tasks[n], Transport) else self.tasks[n].pos[0]),
              str(task.goal[1] if isinstance(self.tasks[n], Transport) else self.tasks[n].pos[1]),
              str(self.navigator.plan(self.pos, task.pos).get_distance()),
              str(task.priority),
              str(self.jobs[n].burst_time.seconds/(self.config.stop - self.config.start).seconds*self.slot_num_full/self.slot_num),
              '\n',
            ]))
    self.close()
