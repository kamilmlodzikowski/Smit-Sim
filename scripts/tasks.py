import numpy as np
import random
import gym
import rospy
from navfn.srv import MakeNavPlan
import geometry_msgs.msg

class Task(object):
    def __init__(self):
        self.age = 0

    def wait(self, dt):
        self.age = self.age + dt
        self.do_wait(dt)

    def work(self, dt):
        self.age = self.age + dt
        self.do_work(dt)

    def do_estimate(self):
        raise NotImplementedError()

    def do_wait(self, dt):
        raise NotImplementedError()

    def do_work(self, dt):
        raise NotImplementedError()

    def is_alive(self):
        raise NotImplementedError()

class Empty(Task):
    def __init__(self):
        super(Empty, self).__init__()

    def do_estimate(self):
        return 0

    def do_wait(self, dt):
        pass

    def do_work(self, dt):
        pass

    def is_alive(self):
        return True

    def __str__(self):
        return 'E'

    __repr__ = __str__

# Transport z punkt A do B
class Transport(Task):
    def __init__(self, pt1, pt2, spd):
        super(Transport, self).__init__()
        self.pos = np.array(pt1)
        self.goal = np.array(pt2)
        self.spd = spd

    def do_estimate(self):
        return np.linalg.norm(self.goal - self.pos)

    def do_wait(self, dt):
        pass

    def do_work(self, dt):
        dist = np.linalg.norm(self.goal - self.pos)

        if dist < self.spd * dt:
            self.pos = self.goal
        else:
            v = (self.goal - self.pos) / dist
            self.pos = self.pos + v * self.spd * dt

    def is_alive(self):
        return True

    def __str__(self):
        dst = self.do_estimate()
        return 'T | dst: ' + str(dst*0.001) + ' m | spd: ' + str(self.spd*0.001) +' m/s'

    __repr__ = __str__

class Drive(Task):
    globalPose = np.array([0, 0])
    def __init__(self, goal, spd):
        super(Drive, self).__init__()
        self.localPose = Drive.globalPose
        self.goalPose = np.array(goal)
        self.spd = spd

        rospy.wait_for_service('/planner/make_plan')
        self.planner = rospy.ServiceProxy('/planner/make_plan', MakeNavPlan)

        self.req1 = geometry_msgs.msg.PoseStamped()
        self.req1.pose.orientation.w = 1

        self.req2 = geometry_msgs.msg.PoseStamped()
        self.req2.pose.position.x = self.goalPose[0]
        self.req2.pose.position.y = self.goalPose[1]
        self.req2.pose.orientation.w = 1

        self.get_plan()

    def get_plan(self):
        self.req1.pose.position.x = self.localPose[0]
        self.req1.pose.position.y = self.localPose[1]
        self.resp = self.planner(self.req1, self.req2)
        self.path = np.array([[point.pose.position.x, point.pose.position.y] for point in self.resp.path])
        self.distances = [np.linalg.norm(self.path[0] - self.localPose)] + [np.linalg.norm(self.path[i+1] - self.path[i]) for i in range(len(self.path)-1)]
        
    def do_estimate(self):
        if (self.localPose == Drive.globalPose).all():
            return sum(self.distances)
        else:
            return np.linalg.norm(Drive.globalPose - self.localPose) + sum(self.distances)

    def do_wait(self, dt):
        pass

    def do_work(self, dt):
        if (self.localPose != Drive.globalPose).all():
            self.localPose = Drive.globalPose
            self.get_plan() 
        dst = self.spd*dt
        while dst > 0:
            if dst > self.distances[0]:
                dst -= self.distances[0]
                self.distances = self.distances[1:]
                self.localPose = self.path[0]
                self.path = self.path[1:]
            else:
                self.localPose = self.localPose + dst/self.distances[0] * (self.path[0] - self.localPose)
                self.distances[0] -= dst
                dst = 0

        Drive.globalPose = self.localPose

    def is_alive(self):
        return True

    def __str__(self):
        return 'D | dst: ' + str(self.do_estimate()) + ' m | spd: ' + str(self.spd) + ' m/s'

    __repr__ = __str__

# Upadek w punkcie A
class Fall(Task):
    def __init__(self, pt, urgency):
        super(Fall, self).__init__()
        self.pos = pt
        self.urgency = urgency

    def do_estimate(self):
        return self.urgency

    def do_wait(self, dt):
        if self.urgency > 0:
            self.urgency = self.urgency + dt

    def do_work(self, dt):
        self.urgency = max(self.urgency - 10*dt, 0)

    def is_alive(self):
        return self.urgency < 60*60

    def __str__(self):
        return 'F | urg: ' + str(self.urgency)

    __repr__ = __str__

def EmptyGenerator():
  return Empty()

def TransportGenerator():
    x_min = 0
    x_max = 10 * 1000
    y_min = 0
    y_max = 10 * 1000
    x1 = x_min + random.random() * (x_max - x_min)
    x2 = x_min + random.random() * (x_max - x_min)
    y1 = y_min + random.random() * (y_max - y_min)
    y2 = y_min + random.random() * (y_max - y_min)
    spd_min = 0.1 * 1000
    spd_max = 1.0 * 1000
    spd = spd_min + random.random() * (spd_max - spd_min)
    return Transport([x1, y1], [x2, y2], spd)

def FallGenerator():
    x_min = 0
    x_max = 10 * 1000
    y_min = 0
    y_max = 10 * 1000
    x1 = x_min + random.random() * (x_max - x_min)
    y1 = y_min + random.random() * (y_max - y_min)
    urg_min = 10*60
    urg_max = 55*60
    urg = urg_min + random.random() * (urg_max - urg_min)
    return Fall([x1, y1], urg)

class Tasks(gym.Env):
  def __init__(self, task_desc, N, penalty):
    self.tasks = []
    self.selected = -1
    self.dt = 15
    self.penalty = penalty
    self.max_steps = 240
    self.steps = 0

    self.N = N
    self.task_desc = task_desc
    state_shape = (len(task_desc), N)
    self.state = np.zeros(state_shape)
    self.state_space = gym.spaces.Box(low=0, high=100000, shape=state_shape)
    self.action_space = gym.spaces.Discrete(len(task_desc)*N)

    self.switching_penalty = penalty
    
    self.reset()

  def render(self, mode="save"):
    for i,t in enumerate(self.tasks):
      t_id = i // self.N
      t_no = i % self.N
      print(t_id, t_no, t)

  def reset(self):
    self.tasks = []
    self.steps = 0
    for i,t in enumerate(self.task_desc):
      for i in range(self.N):
        task = t()
        if random.random() > 0.5:
          task = Empty()
        self.tasks.append(task)

    self.switching_counter = 0

    for i,t in enumerate(self.tasks):
      t_id = i // self.N
      t_no = i % self.N
      self.state[t_id, t_no] = t.do_estimate()

    return self.state

  def do_step(self, action):
    new_state = np.zeros_like(self.state)
    self.steps = self.steps + 1
    for i,t in enumerate(self.tasks):
      t_id = i // self.N
      t_no = i % self.N

      if i == action:
        t.do_work(self.dt)

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

    if (action != self.selected) and (self.selected != -1):
      for i in range(self.penalty):
        new_state = self.do_step(-1)

    self.selected = action

    # big penalty for allowing a task to die or working too long
    if not self.is_alive() or self.steps >= self.max_steps:
      done = True
      reward = -10000
    else:
      # penalty for choosing non-existing or finished task
      if self.tasks[action].do_estimate() <= 0:
        reward = -10000

      new_state = self.do_step(action)

      # bonus reward for finishing all tasks
      if np.sum(new_state) == 0:
        done = True
        reward = 5000

      # reward for finishing single task
      if self.tasks[action].do_estimate() <= 0 and reward == 0:
        reward = 2000

      # bonus for selecting actual task
      if reward == 0:
        reward = 1000

      reward = reward + np.sum(self.state - new_state)
    
    reward = 0.001 * reward
    self.state = new_state
    return self.state, reward, done, {}