import random
import numpy as np
import sys
import os
from shutil import copyfile
from datetime import datetime, timedelta, date, time
import math as m
sys.path.insert(0, '../')
from my_tasks import Transport, Empty
from linear_path_ROS_planner import ROSNavigation
sys.path.insert(0, '../../../tasker/src/TaskER/')
from RequestTable import RequestTable, ScheduleRules, ScheduleRule, TaskerReqest
import tensorflow as tf

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
    self.estimator_path = 'estimator_model/save_20'
    self.predictor_path = 'predictor_model/save_100'

class System():
  def __init__(self, task_config, config = SystemConfig()):
    self.config = config
    self.task_config = task_config
    self.tasks = []
    self.now = self.config.start

    self.estimator = tf.keras.models.Sequential([
      tf.keras.layers.Dense(64, activation = 'relu'),
      tf.keras.layers.Dense(32, activation = 'relu'),
      tf.keras.layers.Dense(1, activation = 'linear'),
    ])
    # self.estimator.compile()
    self.estimator.load_weights(self.config.estimator_path)

    self.predictor = tf.keras.models.Sequential([
      tf.keras.layers.Bidirectional(tf.keras.layers.LSTM(64)),
      tf.keras.layers.Dense(36),
    ])
    # self.predictor.compile()
    self.predictor.load_weights(self.config.predictor_path)
    
    self.slot_num = int(self.config.time_horizon/self.config.time_slot)
    self.slot_num_full = int((self.config.stop - self.config.start)/self.config.time_slot)
    self.time_to_horizon = self.slot_num_full/self.slot_num/(self.config.stop - self.config.start).seconds
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
    self.time_eval = (self.now - self.config.start).seconds*self.time_to_horizon
    for i, t in enumerate(self.tasks):
      sr = ScheduleRules()
      sr.addRule(ScheduleRule(rule_type='at', rule_value=t.getDeadline()))
      job = TaskerReqest(ID=t.getID(),huid=t.getUUID(), plan_args='', req_time=self.now, shdl_rules=sr, priority=t.getPriority())
      # job.set_burst_time(t.getBurst() + timedelta(seconds = self.navigator.plan(self.pos, t.pos).get_distance() / self.config.robot_speed))
      # print(np.array(self.estimator(np.expand_dims(np.array([
      #                     self.time_eval,
      #                     self.config.day,
      #                     (t.getDeadline() - t.getBurst() - self.config.start).seconds*self.time_to_horizon,
      #                     t.pos[0],
      #                     t.pos[1],
      #                     t.goal[0] if isinstance(t, Transport) else t.pos[0],
      #                     t.goal[1] if isinstance(t, Transport) else t.pos[1],
      #                     self.navigator.plan(self.pos, t.pos).get_distance(),
      #                     t.getPriority(),
      #                   ]), axis = 0)))[0,0])
      job.set_burst_time(timedelta(seconds = np.array(self.estimator(np.expand_dims(np.array([
                          self.time_eval,
                          self.config.day,
                          (t.getDeadline() - t.getBurst() - self.config.start).seconds*self.time_to_horizon,
                          t.pos[0],
                          t.pos[1],
                          t.goal[0] if isinstance(t, Transport) else t.pos[0],
                          t.goal[1] if isinstance(t, Transport) else t.pos[1],
                          self.navigator.plan(self.pos, t.pos).get_distance(),
                          t.getPriority(),
                        ]), axis = 0)))[0,0] * self.config.time_horizon.seconds))
      job.evaluate_rules()
      self.rt.addRecord(job)
      self.jobs.append(job)
    self.out, self.profit = self.rt.schedule_with_priority()

    self.state = np.zeros(self.state_shape)
    self.predicted_state = np.zeros(self.state_shape)
    self.empty_tasks = []
    self.empty_jobs = []
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
            t = self.tasks[n]
            if t.do_estimate() and job.start_time < self.now + self.config.time_slot and job.start_time > self.now - self.config.time_slot:
              job.set_burst_time(timedelta(seconds = np.array(self.estimator(np.expand_dims(np.array([
                            self.time_eval,
                            self.config.day,
                            (t.getDeadline() - t.getBurst() - self.config.start).seconds*self.time_to_horizon,
                            t.pos[0],
                            t.pos[1],
                            t.goal[0] if isinstance(t, Transport) else t.pos[0],
                            t.goal[1] if isinstance(t, Transport) else t.pos[1],
                            self.navigator.plan(self.pos, t.pos).get_distance(),
                            t.getPriority(),
                          ]), axis = 0)))[0,0] * self.config.time_horizon.seconds))
              job.evaluate_rules()
              self.rt.updateRecord(job)
          self.out, self.profit = self.rt.schedule_with_priority()
        else:
          print('Worked on job ' + j.jobID + ': ' + str(self.tasks[int(j.jobID)].do_estimate()))
        break

    # continue work
    self.now = self.now + self.dt
    self.step_until(deadline)

  def estimate_tasks_duration(self):
    self.time_eval = (self.now - self.config.start).seconds*self.time_to_horizon
    for i, t in enumerate(self.tasks):
      if t.do_estimate():
        self.jobs[i].set_burst_time(timedelta(seconds = np.array(self.estimator(np.expand_dims(np.array([
                            self.time_eval,
                            self.config.day,
                            (t.getDeadline() - t.getBurst() - self.config.start).seconds*self.time_to_horizon,
                            t.pos[0],
                            t.pos[1],
                            t.goal[0] if isinstance(t, Transport) else t.pos[0],
                            t.goal[1] if isinstance(t, Transport) else t.pos[1],
                            self.navigator.plan(self.pos, t.pos).get_distance(),
                            t.getPriority(),
                          ]), axis = 0)))[0,0] * self.config.time_horizon.seconds))
        self.jobs[i].evaluate_rules()
        self.rt.updateRecord(self.jobs[i])

  def getState(self):
    for s in range(self.slot_num):
      start = self.now + s*self.config.time_slot
      stop = self.now + (s+1)*self.config.time_slot
      for i,j in enumerate(self.jobs):
        if j.start_time >= start and j.start_time < stop or j.deadline > start and j.deadline <= stop or j.start_time < start and j.deadline > stop:
          self.state[s*3] += j.priority
          self.state[2+s*3] += 1
      for i,j in enumerate(self.out.scheduled):
        if j.start >= start and j.start < stop or j.stop > start and j.stop <= stop or j.start < start and j.stop > stop:
          self.state[1+s*3] += self.rt.get_request(j.jobID).priority
    return self.state

  def predict_schedule(self):
    if len(self.empty_jobs):
      for j in self.empty_jobs:
        rt.removeRecord_by_id(j.jobID)

    self.empty_tasks = []
    self.empty_jobs = []
    self.predicted_state = np.array(self.predictor(np.expand_dims(np.expand_dims(np.concatenate((np.array([(self.now - self.config.start).seconds*self.time_to_horizon, self.config.day]), self.state)), axis = 0), axis = 0))[0])

    for s in range(self.slot_num - 1):
      task_diff = round(self.predicted_state[s*3] - self.state[(s+1)*3])
      if task_diff > 0:
        print(f'Adding {task_diff} empty_tasks')
        task_priorities = round((self.predicted_state[s*3 + 2] - self.state[(s+1)*3 + 2])/task_diff)
        for i in range(task_diff):
          t = Empty(self.now + (s+1)*self.config.time_horizon, task_priorities)
          self.empty_tasks.append(t)
          sr = ScheduleRules()
          sr.addRule(ScheduleRule(rule_type='at', rule_value=t.getDeadline()))
          job = TaskerReqest(ID=t.getID(),huid=t.getUUID(), plan_args='', req_time=self.now, shdl_rules=sr, priority=t.getPriority())
          job.set_burst_time(self.config.time_slot)
          job.evaluate_rules()
          self.rt.addRecord(job)
          self.empty_jobs.append(job)

    if len(self.empty_jobs):
      self.out, self.profit = self.rt.schedule_with_priority()

  def run_env(self):
    while(self.now < self.config.stop):
      print("Stepping from " + str(self.now) + " to " + str(self.now + self.config.time_slot))
      self.estimate_tasks_duration()
      self.predict_schedule()
      self.step_until(self.now + self.config.time_slot)

    # self.close()
