import numpy as np
import random
import sys
sys.path.insert(0, '../')
sys.path.insert(0, '../../../dqn')
from system import System, SystemConfig
from linear_path_ROS_planner import ROSNavigation

class MySystem(System):
  """docstring for MySystem"""
  def __init__(self, task_config, config = SystemConfig(), random_state = -1):
      super(MySystem, self).__init__(task_config, config, random_state)
      self.navigator = ROSNavigation()
        
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
          [self.pos, time_left] = path.step(self.config.robot_speed, self.dt)

          # force the whole step wait
          time_left = 0

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

      new_state[t_id, t_no, 0] = t.do_estimate()
      new_state[t_id, t_no, 1] = t.dist(self.pos)

    return new_state