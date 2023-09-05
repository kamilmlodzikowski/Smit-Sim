import numpy as np
import random
import sys
from linear_path_ROS_planner import ROSNavigation
from datetime import datetime, timedelta
import time
from time import sleep

class Task:
    id_counter = 0
    def __init__(self):
        self.id = str(Task.id_counter)
        Task.id_counter += 1
        self.age = 0
        self.pos = np.array([0, 0])
        self.preemptive = True

    def wait(self, dt):
        self.age = self.age + dt
        self.do_wait(dt)

    def work(self, dt):
        self.age = self.age + dt
        self.do_work(dt)

    def getID(self):
        return self.id

    def getUUID(self):
        raise NotImplementedError()

    def getPriority(self):
        raise NotImplementedError()

    def getDeadline(self):
        raise NotImplementedError()

    def setDeadline(self):
        raise NotImplementedError()

    def getBurst(self):
        raise NotImplementedError()

    def setBurst(self):
        raise NotImplementedError()

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

    def is_alive(self, now):
        raise NotImplementedError()

    def getDeathTime(self):
        raise NotImplementedError()

class Empty(Task):
    uuid_counter = 0
    def __init__(self, deadline, priority = 0):
        super().__init__()
        self.uuid = 'transport_' + str(Empty.uuid_counter)
        Empty.uuid_counter+= 1
        self.priority = priority
        self.deadline = deadline

    def getUUID(self):
        return self.uuid

    def getPriority(self):
        return self.priority

    def getDeadline(self):
        return self.deadline


# Transport z punkt A do B
class Transport(Task):
    navigator = ROSNavigation()
    uuid_counter = 0
    def __init__(self, deadline, calltime, pt1, pt2, spd):
        super().__init__()
        self.uuid = 'transport_' + str(Transport.uuid_counter)
        Transport.uuid_counter+= 1
        self.priority = 1
        self.deadline = deadline
        self.calltime = calltime
        self.pos = np.array(pt1)
        self.goal = np.array(pt2)
        self.spd = spd
        self.path = Transport.navigator.plan(self.pos, self.goal)

    def getUUID(self):
        return self.uuid

    def getPriority(self):
        return self.priority

    def getDeadline(self):
        return self.deadline

    def setDeadline(self, new_deadline):
        self.deadline = new_deadline

    def getBurst(self):
        return timedelta(seconds = self.path.get_distance()/self.spd)

    def setBurst(self, new_burst):
        self.spd = self.path.get_distance()/new_burst.seconds

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

    def is_alive(self, now):
        return True

    def getDeathTime(self):
        return 0

    def __str__(self):
        dst = self.do_estimate()
        return f'D | dst: {dst:.2f} m | spd: {self.spd:.2f} m/s'

    __repr__ = __str__
    
    def serialize(self):
      return f"{self.goal[0]}|{self.goal[1]}"

# Upadek w punkcie A
class Fall(Task):
    uuid_counter = 0
    def __init__(self, deadline, calltime, pt, urgency):
        super().__init__()
        self.uuid = 'fall_' + str(Fall.uuid_counter)
        Fall.uuid_counter+= 1
        self.priority = 2
        self.deadline = deadline
        self.calltime = calltime
        self.pos = np.array(pt)
        self.urgency = urgency
        self.preemptive = False

    def getUUID(self):
        return self.uuid

    def getPriority(self):
        return self.priority

    def getDeadline(self):
        return self.deadline

    def setDeadline(self, new_deadline):
        self.deadline = new_deadline

    def getBurst(self):
        return timedelta(seconds=self.urgency)

    def setBurst(self, new_burst):
        self.urgency = new_burst.seconds
    
    def dist(self, pos):
        return np.linalg.norm(pos - self.pos)

    def updatePos(self, pos):
        return self.pos

    def do_estimate(self):
        return self.urgency

    def do_wait(self, dt):
        pass

    def do_work(self, dt):
        self.urgency = max(self.urgency - dt, 0)

    def is_alive(self, now):
        if self.urgency:
            return now < self.deadline + timedelta(minutes=15)
        else:
            return True

    def getDeathTime(self):
        return self.deadline + timedelta(minutes=15)

    def __str__(self):
        return f'F | urg: {self.urgency:.0f}'

    __repr__ = __str__
    
    def serialize(self):
      return f""


def TransportGenerator(now, time_horizon, spawn_zones):
    # define absolutes
    x_min = min([zone[0][0] for zone in spawn_zones])
    x_max = max([zone[0][1] for zone in spawn_zones])
    y_min = min([zone[1][0] for zone in spawn_zones])
    y_max = max([zone[1][1] for zone in spawn_zones])
    # initialize positions
    x1 = x_min + random.random() * (x_max - x_min)
    y1 = y_min + random.random() * (y_max - y_min)
    x2 = x_min + random.random() * (x_max - x_min)
    y2 = y_min + random.random() * (y_max - y_min)
    # regenerate until proper start positions are found
    while(True):
        in_room = False
        for zone in spawn_zones:
            if x1 >= zone[0][0] and x1 <= zone[0][1] and y1 >= zone[1][0] and y1 <= zone[1][1]:
                # print(f'{x1} in {zone}')
                in_room = True
                break
        # if position is inside a room exit loop
        if in_room:
            break
        # generate new posiitons
        x1 = x_min + random.random() * (x_max - x_min)
        y1 = y_min + random.random() * (y_max - y_min)
    # regenerate until proper stop positions are found
    while(True):
        in_room = False
        for zone in spawn_zones:
            if x2 >= zone[0][0] and x2 <= zone[0][1] and y2 >= zone[1][0] and y2 <= zone[1][1]:
                # print(f'{x2} in {zone}')
                in_room = True
                break
        # if position is inside a room exit loop
        if in_room:
            break
        # generate new posiitons
        x2 = x_min + random.random() * (x_max - x_min)
        y2 = y_min + random.random() * (y_max - y_min)
    spd_min = 0.01
    spd_max = 0.1
    spd = spd_min + random.random() * (spd_max - spd_min)
    deadline = now + random.random() * time_horizon
    calltime = now + time_horizon
    while calltime > deadline - timedelta(seconds = 5):
        calltime = now + random.random() * time_horizon - timedelta(seconds = 5)
    return Transport(deadline, calltime, [x1, y1], [x2, y2], spd)

def FallGenerator(now, time_horizon, spawn_zones):
    # define absolutes
    x_min = min([zone[0][0] for zone in spawn_zones])
    x_max = max([zone[0][1] for zone in spawn_zones])
    y_min = min([zone[1][0] for zone in spawn_zones])
    y_max = max([zone[1][1] for zone in spawn_zones])
    # initialize positions
    x1 = x_min + random.random() * (x_max - x_min)
    y1 = y_min + random.random() * (y_max - y_min)
    # regenerate until proper positions are found
    while(True):
        in_room = False
        for zone in spawn_zones:
            if x1 >= zone[0][0] and x1 <= zone[0][1] and y1 >= zone[1][0] and y1 <= zone[1][1]:
                in_room = True
                break
        # if position is inside a room exit loop
        if in_room:
            break
        # generate new posiitons
        x1 = x_min + random.random() * (x_max - x_min)
        y1 = y_min + random.random() * (y_max - y_min)
    urg_min = 60
    urg_max = 300
    urg = urg_min + random.random() * (urg_max - urg_min)
    deadline = now + random.random() * time_horizon
    calltime = now + time_horizon
    while calltime > deadline - timedelta(seconds = 5):
        calltime = now + random.random() * time_horizon - timedelta(seconds = 5)
    return Fall(deadline, calltime, [x1, y1], urg)

class TaskConfig(object):
    def __init__(self, task_desc, count, now, time_horizon, seed = -1, random_task_count = 0, deadline_variation = 0, burst_variation = 0, randomize_call_time = False):
        self.types = len(task_desc)
        self.count = count
        self.rcount = random_task_count
        self.task_desc = task_desc
        self.now = now
        self.time_horizon = time_horizon
        if seed >= 0:
            self.fix_random = True
            self.seed = seed
        else:
            self.fix_random = False
            self.seed = -1
        self.b_var = burst_variation
        self.d_var = deadline_variation
        self.random_call = randomize_call_time

        # self.task_prob = task_prob

    def generate(self, spawn_zones = [((1,9),(1,9))]):
        Task.id_counter = 0
        Fall.uuid_counter = 0
        Transport.uuid_counter = 0
        if self.fix_random:
          random.seed(self.seed)
        else:
          seed = random.randint(0, 10000)
          random.seed(seed)
          self.seed = seed
          
        tasks = []
        for i,t in enumerate(self.task_desc):
          for i in range(self.count):
            task = t(self.now, self.time_horizon, spawn_zones)
            print(task.uuid)
            # print(task.pos)
            # print(task.goal)
            # sleep(1)
            tasks.append(task)

        if self.random_call:
            random.seed(time.time())
            for t in tasks:
                calltime = self.now + self.time_horizon
                while calltime > t.deadline - timedelta(seconds = 5):
                    calltime = self.now + random.random() * self.time_horizon - timedelta(seconds = 5)
                t.calltime = calltime

        if self.d_var:
            random.seed(time.time())
            for t in tasks:
                t.setDeadline(t.getDeadline() + random.uniform(-self.d_var, self.d_var) * t.getBurst())

        if self.b_var:
            random.seed(time.time())
            for t in tasks:
                t.setBurst(timedelta(seconds = t.getBurst().seconds + random.uniform(-self.b_var, self.b_var) * t.getBurst().seconds))

        if self.rcount > 0:
            random.seed(time.time())
            for i,t in enumerate(self.task_desc):
              for i in range(self.rcount):
                task = t(self.now, self.time_horizon, spawn_zones)
                print(task.uuid)
                tasks.append(task)

        return tasks