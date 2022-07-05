# from __future__ import print_function
import numpy as np
import random
import gym
from task import Empty
# from linear_path import LinearPath
from linear_path_ROS_planner import ROSNavigation

class Tasks(gym.Env):
  def __init__(self, task_desc, N, substeps = 1, save = False):
    self.tasks = []
    self.dt = 15 / substeps
    self.spd = 0.75
    # self.penalty = penalty
    self.max_steps = 240 * substeps
    self.save = save

    self.N = N
    self.task_desc = task_desc
    state_shape = (len(task_desc), N, 2)
    self.state = np.zeros(state_shape)
    self.state_space = gym.spaces.Box(low=0, high=100000, shape=state_shape)
    self.action_space = gym.spaces.Discrete(len(task_desc)*N)

    self.navigator = ROSNavigation()

    
    self.reset()


  def render(self, mode="save"):
    if mode=="human" and self.save:
      out = str(self.selected) + ";" + str(self.pos[0]) + ";" + str(self.pos[1]) + ";"
      for i,t in enumerate(self.tasks):
        t_id = i // self.N
        t_no = i % self.N
        out = out + str(t_id) + ";" + str(t_no) + ";" + str(t.pos[0]) + ";" + str(t.pos[1]) + ";" + str(t.serialize()) + ";"
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

  def reset(self):
    self.selected = -1
    self.tasks = []
    self.steps = 0
    for i,t in enumerate(self.task_desc):
      for i in range(self.N):
        task = t()
        if random.random() > 0.5:
          task = Empty()
        self.tasks.append(task)

    # initial agent position
    x_min = 1
    x_max = 50
    y_min = 1
    y_max = 30
    x1 = x_min + random.random() * (x_max - x_min)
    y1 = y_min + random.random() * (y_max - y_min)
    self.pos = np.array([x1, y1])

    for i,t in enumerate(self.tasks):
      t_id = i // self.N
      t_no = i % self.N
      self.state[t_id, t_no, 0] = t.do_estimate()
      self.state[t_id, t_no, 1] = t.dist(self.pos)

    print("Reset")
  
    if self.save:
      from datetime import datetime
      now = datetime.now() # current date and time
      fname = now.strftime("%Y%m%d_%H%M%S.csv")
      self.file_out = open(fname, "w")

    return self.state

  def do_step(self, action):
    new_state = np.zeros_like(self.state)
    self.steps = self.steps + 1
    for i,t in enumerate(self.tasks):
      t_id = i // self.N
      t_no = i % self.N

      if i == action:
        time_left = self.dt

        # do the actual travel to the action spot if distanse from the agent is greater than threshold
        if t.dist(self.pos) > 0.5:
          # path = LinearPath(self.pos, [t.pos])
          path = self.navigator.plan(self.pos, t.pos)
          [self.pos, time_left] = path.step(self.spd, self.dt)
          # task waits when the agent moves
          t.do_wait(self.dt - time_left)

        # travel took less than single time step - do part of the task
        if time_left > 0:
          t.do_work(time_left)
          self.pos = t.updatePos(self.pos)

        # task finished
        if t.do_estimate() <= 0:
            self.selected = -1
      else:
        t.do_wait(self.dt)

      new_state[t_id, t_no] = t.do_estimate()

    return new_state

  def is_alive(self):
    for t in self.tasks:
      if not t.is_alive():
        return False

    return True

  def step(self, action):
    new_state = np.zeros_like(self.state)
    done = False
    reward = 0

    # old behaviour - fixed penalty for changing the task
    # if (action != self.selected) and (self.selected != -1):
    #   for i in range(self.penalty):
    #     new_state = self.do_step(-1)

    # new behaviour - calculating the actual cost (time) to travel to a new task
    # implemented inside the `do_step` method

    self.selected = action

    # big penalty for allowing a task to die or working too long
    if not self.is_alive() or self.steps >= self.max_steps:
      done = True
      reward = -20
    else:
      # penalty for choosing non-existing or finished task
      if self.tasks[action].do_estimate() <= 0:
        reward = -10

      new_state = self.do_step(action)

      # bonus reward for finishing all tasks
      if np.sum(new_state) == 0:
        done = True
        reward = 5

      # reward for finishing single task
      if self.tasks[action].do_estimate() <= 0 and reward == 0:
        reward = 2

      # bonus for selecting actual task
      if reward == 0:
        reward = 1

      reward = reward + np.sum(self.state - new_state)
    
    #reward = 0.001 * reward
    self.state = new_state
    return self.state, reward, done, {}