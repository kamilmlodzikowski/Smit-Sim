import numpy as np
import random
from linear_path_ROS_planner import ROSNavigation

class Task(object):
    def __init__(self):
        self.age = 0
        self.pos = np.array([0, 0])

    def wait(self, dt):
        self.age = self.age + dt
        self.do_wait(dt)

    def work(self, dt):
        self.age = self.age + dt
        self.do_work(dt)

    def dist(self, pos):
        raise NotImplementedError()

    def updatePos(self, pos):
        raise NotImplementedError()

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

    def dist(self, pos):
        return 0
        
    def updatePos(self, pos):
        return pos

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

    def serialize(self):
      return ""

# Transport z punkt A do B
class Transport(Task):
    def __init__(self, pt1, pt2, spd):
        super(Transport, self).__init__()
        self.pos = np.array(pt1)
        self.goal = np.array(pt2)
        self.spd = spd

    def dist(self, pos):
        return np.linalg.norm(pos - self.pos)

    def updatePos(self, pos):
        return self.pos

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
        return 'T | dst: ' + str(dst) + ' m | spd: ' + str(self.spd) +' m/s'

    __repr__ = __str__
    
    def serialize(self):
      return str(self.goal[0]) + "|" + str(self.goal[1])

# Upadek w punkcie A
class Fall(Task):
    def __init__(self, pt, urgency):
        super(Fall, self).__init__()
        self.pos = np.array(pt)
        self.urgency = urgency
    
    def dist(self, pos):
        return np.linalg.norm(pos - self.pos)

    def updatePos(self, pos):
        return self.pos

    def do_estimate(self):
        return self.urgency*0.001

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
    
    def serialize(self):
      return ""

class Drive(Task):
    navigator = ROSNavigation()
    def __init__(self, pos, goal, spd):
        super(Drive, self).__init__()
        self.pos = np.array(pos)
        self.goal = np.array(goal)
        self.spd = spd
        self.path = Drive.navigator.plan(pos, goal)

    def dist(self, pos):
        return np.linalg.norm(pos - self.pos)

    def updatePos(self, pos):
        return self.pos

    def do_estimate(self):
        return self.path.get_distance()

    def do_wait(self, dt):
        pass

    def do_work(self, dt):
        self.path.step(self.spd, dt)
        self.pos = self.path.pos

    def is_alive(self):
        return True

    def __str__(self):
        return 'D | dst: ' + str(self.do_estimate()) + ' m | spd: ' + str(self.spd) + ' m/s'

    __repr__ = __str__
    
    def serialize(self):
      return str(self.goal[0]) + "|" + str(self.goal[1])

def EmptyGenerator():
  return Empty()

def TransportGenerator():
    x_min = 1
    x_max = 50
    y_min = 1
    y_max = 30
    x1 = x_min + random.random() * (x_max - x_min)
    x2 = x_min + random.random() * (x_max - x_min)
    y1 = y_min + random.random() * (y_max - y_min)
    y2 = y_min + random.random() * (y_max - y_min)
    spd_min = 0.1
    spd_max = 1.0
    spd = spd_min + random.random() * (spd_max - spd_min)
    return Transport([x1, y1], [x2, y2], spd)

def FallGenerator():
    x_min = 1
    x_max = 50
    y_min = 1
    y_max = 30
    x1 = x_min + random.random() * (x_max - x_min)
    y1 = y_min + random.random() * (y_max - y_min)
    urg_min = 10*60
    urg_max = 55*60
    urg = urg_min + random.random() * (urg_max - urg_min)
    return Fall([x1, y1], urg)

def DriveGenerator():
    x_min = 1
    x_max = 50
    y_min = 1
    y_max = 30
    x1 = x_min + random.random() * (x_max - x_min)
    x2 = x_min + random.random() * (x_max - x_min)
    y1 = y_min + random.random() * (y_max - y_min)
    y2 = y_min + random.random() * (y_max - y_min)
    spd_min = 0.1
    spd_max = 1.0
    spd = spd_min + random.random() * (spd_max - spd_min)
    return Drive([x1, y1], [x2, y2], spd)
