import gym
import random
import numpy as np
import sys
from datetime import datetime, timedelta, date, time
import math as m
sys.path.insert(0, '../')
from linear_path_ROS_planner import ROSNavigation
sys.path.insert(0, '../../../tasker/src/TaskER/')
from RequestTable import RequestTable, ScheduleRules, ScheduleRule, TaskerReqest

class SystemConfig(object):
  def __init__(self):
    self.robot_speed = 0.1

    # self.reward_existing_task = 1
    # self.reward_finish_task = 2
    self.reward_finish_all = 10

    # self.penalty_switch = 2
    # self.penalty_wrong_task = 10
    self.penalty_dead = 20
    self.penalty_task_wont_finish = 10

    self.dt = timedelta(seconds = 60)
    self.time_horizon = timedelta(hours = 1)
    self.now = datetime.combine(date.today(), time(15, 0))
    self.time_slot = timedelta(minutes = 5)
    self.recalculation_time = timedelta(minutes = (5))

    # self.weight_estimate = 1.0
    # self.weight_distance = 1.0
    # self.weight_current = 1.0

    self.save = False
    self.prefix = ""
    self.beta = 0.1


class System(gym.Env):
  def __init__(self, task_config, config = SystemConfig(), random_state = -1):
    self.config = config
    self.task_config = task_config
    self.tasks = []
    self.dt = self.config.dt
    self.now = self.config.now
    self.alpha = -1/(self.config.time_horizon.seconds*self.config.time_horizon.seconds)
    
    # self.penalty = penalty
    # self.max_steps = 2*self.config.time_horizon.seconds/self.config.recalculation_time
    self.save = config.save
    self.random_state = random_state
    if random_state >= 0:
      self.fix_random = True
    else:
      self.fix_random = False

    self.N = task_config.count
    self.slot_num = int(self.config.time_horizon/self.config.time_slot)
    state_shape = (5 + 3 * self.slot_num)
    self.state = np.zeros(state_shape)
    self.state_space = gym.spaces.Box(low=-100, high=100, shape=(state_shape,))
    # self.action_space = gym.spaces.Box(low = 0, high = 1, shape = (2,))
    self.action_space = gym.spaces.Box(low = 0, high = 1, shape = (1,))

    self.navigator = ROSNavigation()
    self.reset()


  def render(self, mode="save"):
    if (mode=="human" or mode=="save") and self.save:
      out = f"{self.selected};{self.pos[0]};{self.pos[1]};"
      for i,t in enumerate(self.tasks):
        t_id = i // self.N
        t_no = i % self.N
        out = out + f"{t_id};{t_no};{t.do_estimate()};{t.pos[0]};{t.pos[1]};{t.serialize()};"
      self.file_out.write(out + "\n")
    elif mode == "ansi":
      arr = []
      for i in range(20):
        arr.append([-1] * 20)
      for i,t in enumerate(self.tasks):
        t_id = i // self.N
        t_no = i % self.N
        print(t_id, t_no, t)

        if t.do_estimate() > 0:
          x = int(t.pos[0] * 2)
          y = int(t.pos[1] * 2)
          arr[x][y] = t_id

      x = int(self.pos[0] * 2)
      y = int(self.pos[1] * 2)
      arr[x][y] += 100

      for r in arr:
        for c in r:
          if c >= 50:
            print('>', end='')
            c = c - 100
          else:
            print(' ', end='')

          if c >= 0:
            print(str(c%100), end='')
          else:
            print('.', end='')
        print('')

  def close(self):
    if self.save:
      self.file_out.close()

  def reset(self):
    # self.selected = -1
    self.steps = 0
    self.now = self.config.now
    self.tasks = self.task_config.generate()

    # initial agent position
    x_min = 0
    x_max = 10
    y_min = 0
    y_max = 10
    x1 = x_min + random.random() * (x_max - x_min)
    y1 = y_min + random.random() * (y_max - y_min)
    self.pos = np.array([x1, y1])
    self.area = x_max * y_max

    # initialize tasker
    self.rt = RequestTable()
    self.jobs = []
    for i, t in enumerate(self.tasks):
      if i == 0:
        t.deadline = self.now + timedelta(minutes=2)
      sr = ScheduleRules()
      sr.addRule(ScheduleRule(rule_type='at', rule_value=t.getDeadline()))
      job = TaskerReqest(ID=t.getID(),huid=t.getUUID(), plan_args='', req_time=self.now, shdl_rules=sr, priority=t.getPriority())
      job.set_burst_time(t.getBurst() + timedelta(seconds = self.navigator.plan(self.pos, t.pos).get_distance() / self.config.robot_speed))
      job.evaluate_rules()
      self.rt.addRecord(job)
      self.jobs.append(job)
    self.out, self.profit = self.rt.schedule_with_priority()

    # initialize state
    self.proccesed = 0
    self.state = np.zeros(5 + 3 * self.slot_num)
    th = self.config.time_horizon
    b_all = self.jobs[self.proccesed].burst_time
    b = self.tasks[self.proccesed].getBurst()
    dl = self.tasks[self.proccesed].deadline
    dt = self.tasks[self.proccesed].getDeathTime()
    self.state[0:5] = [
      self.tasks[self.proccesed].priority,
      b_all.seconds/th.seconds,
        (dl - b_all - self.now if dl - b_all >= self.now else self.now - (dl - b_all)).seconds/th.seconds,
      (b_all.seconds - b.seconds) * self.config.robot_speed/self.area,
      1 if dt == 0 else (dt - self.now).seconds/th.seconds
    ]

    for s in range(4):
      start = self.now + s*self.config.time_slot
      stop = self.now + (s+1)*self.config.time_slot
      for i,j in enumerate(self.jobs):
        if j.start_time >= start and j.start_time < stop or j.deadline > start and j.deadline <= stop or j.start_time < start and j.deadline > stop:
          self.state[5+s*3] += j.priority
          self.state[7+s*3] += 1
      for i,j in enumerate(self.out.scheduled):
        if j.start >= start and j.start < stop or j.stop > start and j.stop <= stop or j.start < start and j.stop > stop:
          self.state[6+s*3] += self.rt.get_request(j.jobID).priority

    print("Reset")
    # print(self.state)
  
    self.fname = ""
    if self.save:
      now = datetime.now() # current date and time
      fname = self.config.prefix + now.strftime(f"%Y%m%d_%H%M%S_%f_{self.task_config.seed}.csv")
      self.fname = fname
      self.file_out = open(fname, "w")

    return self.state

  def do_step_until(self, deadline):
    print(str(self.now) + " : " + str(deadline))

    if self.now > deadline:
      print('Deadline reached')
      return

    work_done = False

    for i,j in enumerate(self.out.scheduled):

      # print(j.jobID + ' -> ' + str(j.start) + ':' + str(j.stop))

      if j.start <= self.now + self.dt and j.stop > self.now:

        if j.start >= self.now:
          time_left = self.dt.seconds - (j.start - self.now).seconds
        else:
          time_left = self.dt.seconds

        # do the actual travel to the action spot if distanse from the agent is greater than threshold
        if self.tasks[int(j.jobID)].dist(self.pos) > 0.1:
          path = self.navigator.plan(self.pos, self.tasks[int(j.jobID)].pos)
          [self.pos, time_left] = path.step(self.config.robot_speed, time_left)
          # print('Driving to...')
          # print(self.pos)
          # print(self.tasks[int(j.jobID)].pos)

        # work on a task
        if time_left > 0:
          self.tasks[int(j.jobID)].do_work(time_left)
          self.pos = self.tasks[int(j.jobID)].pos
          # print('Working...')

        # update time
        self.now = self.now + self.dt
        work_done = True

        # if task is done remove it from TaskER
        if not self.tasks[int(j.jobID)].do_estimate():
          self.rt.removeRecord_by_id(j.jobID)
          print('Job ' + j.jobID + ' complete')
          self.out, self.profit = self.rt.schedule_with_priority()
          return
        print('Worked on job ' + j.jobID + ': ' + str(self.tasks[int(j.jobID)].do_estimate()))

        break

    # continue work
    if work_done:
      self.do_step_until(deadline)
    else:
      self.now = self.now + self.dt

  def is_alive(self):
    for t in self.tasks:
      if not t.is_alive(self.now):
        return False

    return True

  def all_done(self):
    print(f'All done: {[not bool(t.do_estimate()) for t in self.tasks]}, {sum([not bool(t.do_estimate()) for t in self.tasks])}, {len(self.tasks)}')
    return sum([not bool(t.do_estimate()) for t in self.tasks]) == len(self.tasks)

  # def do_step(self):
  #   self.proccesed += 1
  #   # print(self.proccesed)
  #   if self.proccesed > int(self.jobs[-1].get_id()):
  #     self.proccesed = 0
  #     self.steps += 1
  #     self.do_step_until(self.now + self.config.recalculation_time)
  #   if not self.tasks[self.proccesed].do_estimate():
  #     self.do_step()

  def run_env(self):
    self.do_step_until(self.now + self.config.recalculation_time)

  def get_task_state(self, proccesed):
    self.proccesed = proccesed
    self.state = np.zeros(5 + 3 * self.slot_num)
    th = self.config.time_horizon
    b_all = self.jobs[self.proccesed].burst_time
    b = self.tasks[self.proccesed].getBurst()
    dl = self.tasks[self.proccesed].deadline
    dt = self.tasks[self.proccesed].getDeathTime()
    self.state[0:5] = [
      self.tasks[self.proccesed].priority,
      b_all.seconds/th.seconds,
      (dl - b_all - self.now if dl - b_all >= self.now else self.now - (dl - b_all)).seconds/th.seconds,
      (b_all.seconds - b.seconds) * self.config.robot_speed/self.area,
      1 if dt == 0 else (dt - self.now).seconds/th.seconds
    ]

    for s in range(4):
      start = self.now + s*self.config.time_slot
      stop = self.now + (s+1)*self.config.time_slot
      for i,j in enumerate(self.jobs):
        if j.start_time >= start and j.start_time < stop or j.deadline > start and j.deadline <= stop or j.start_time < start and j.deadline > stop:
          self.state[5+s*3] += j.priority
          self.state[7+s*3] += 1
      for i,j in enumerate(self.out.scheduled):
        if j.start >= start and j.start < stop or j.stop > start and j.stop <= stop or j.start < start and j.stop > stop:
          self.state[6+s*3] += self.rt.get_request(j.jobID).priority

    return self.state

  def step(self, action, proccesed):
    self.proccesed = proccesed
    print(' Action:' + str(action))
    # self.jobs[self.proccesed].priority = action[0]
    burst = self.tasks[self.proccesed].getBurst() + timedelta(seconds = self.navigator.plan(self.pos, self.tasks[self.proccesed].pos).get_distance() / self.config.robot_speed)
    # burst = self.tasks[self.proccesed].getBurst() + timedelta(seconds = self.navigator.plan(self.pos, self.tasks[self.proccesed].pos).get_distance() / self.config.robot_speed)
    old_start = self.jobs[self.proccesed].start_time
    # start_time = self.now + action[1]*self.config.time_horizon
    start_time = self.now + action[0]*self.config.time_horizon
    sr = ScheduleRules()
    sr.addRule(ScheduleRule(rule_type='at', rule_value=start_time + burst))
    self.jobs[self.proccesed].shdl_rules = sr
    self.jobs[self.proccesed].set_burst_time(burst)
    self.jobs[self.proccesed].evaluate_rules()
    # print(self.jobs[self.proccesed].start_time)
    # print(self.jobs[self.proccesed].deadline)

    if not self.tasks[self.proccesed].is_alive(self.now):
      done = True
      reward = -self.config.penalty_dead
      status = "DEAD"
    elif self.now > self.config.now + 2 * self.config.time_horizon:
      done = True
      reward = -self.config.penalty_dead
      status = "TIME"
    elif not self.tasks[self.proccesed].do_estimate():
      done = True
      reward = self.config.reward_finish_all
      status = "DONE"
    else:
      self.rt.updateRecord(self.jobs[self.proccesed])
      old_profit = self.profit
      self.out, self.profit = self.rt.schedule_with_priority()

      R = self.profit - old_profit
      old_start = self.tasks[self.proccesed].deadline - self.jobs[self.proccesed].burst_time
      if old_start > start_time:
        dS = (old_start - start_time).seconds
      else:
        dS = - (start_time - old_start).seconds
      C = self.alpha * dS * dS
      P = 0
      if self.tasks[self.proccesed].getDeathTime() != 0:
        P = - self.config.penalty_task_wont_finish * ((start_time + self.jobs[self.proccesed].burst_time) > self.tasks[self.proccesed].getDeathTime())
      reward = self.config.beta * R + (1 - self.config.beta) * C + P

      self.state = np.zeros(5 + 3 * self.slot_num)
      th = self.config.time_horizon
      b_all = self.jobs[self.proccesed].burst_time
      b = self.tasks[self.proccesed].getBurst()
      dl = self.tasks[self.proccesed].deadline
      dt = self.tasks[self.proccesed].getDeathTime()
      self.state[0:5] = [
        self.tasks[self.proccesed].priority,
        b_all.seconds/th.seconds,
        (dl - b_all - self.now if dl - b_all >= self.now else self.now - (dl - b_all)).seconds/th.seconds,
        (b_all.seconds - b.seconds) * self.config.robot_speed/self.area,
        1 if dt == 0 else (dt - self.now).seconds/th.seconds
      ]

      for s in range(4):
        start = self.now + s*self.config.time_slot
        stop = self.now + (s+1)*self.config.time_slot
        for i,j in enumerate(self.jobs):
          if j.start_time >= start and j.start_time < stop or j.deadline > start and j.deadline <= stop or j.start_time < start and j.deadline > stop:
            self.state[5+s*3] += j.priority
            self.state[7+s*3] += 1
        for i,j in enumerate(self.out.scheduled):
          if j.start >= start and j.start < stop or j.stop > start and j.stop <= stop or j.start < start and j.stop > stop:
            self.state[6+s*3] += self.rt.get_request(j.jobID).priority

      done = False
      status = "WORK"

    return self.state, reward, done, {"status": status, "steps": self.steps, "fname": self.fname}
