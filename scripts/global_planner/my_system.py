import random
import numpy as np
import sys
import os
from shutil import copyfile
from datetime import datetime, timedelta, date, time
import math as m
# sys.path.insert(0, '../')
from my_tasks import Transport, PickAndPlace
from smit_linear_path.linear_path_ROS_planner import ROSNavigation
# sys.path.insert(0, '../../../tasker/src/TaskER/')
from TaskER.RequestTable import RequestTable, ScheduleRules, ScheduleRule, TaskerReqest
import rospy
from smit_sim.srv import GetRoomsAndDoors, GetFurniture, GetObjects, Step, StepRequest
import tensorflow as tf
from train_estimator import get_estimator_model
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from visualization_msgs.msg import Marker

class SystemConfig(object):
  def __init__(self):
    self.robot_speed = 0.5
    self.robot_radius = 0.3
    self.time_horizon = timedelta(hours = 1)
    self.start = datetime.combine(date.today(), time(8, 0))
    self.stop = datetime.combine(date.today(), time(20, 0))
    self.time_slot = timedelta(minutes = 5)
    self.dt = timedelta(seconds = 5)
    self.save = False
    self.prefix = ""
    self.day = 1
    self.estimator_path = 'estimator_model/save_20'
    self.use_estimator = True

class System():
  def __init__(self, task_config, config = SystemConfig()):
    self.config = config
    self.task_config = task_config
    self.tasks = []
    self.now = self.config.start

    if self.config.use_estimator:
      self.estimator = get_estimator_model()
      self.estimator.load_weights(self.config.estimator_path)
    
    self.slot_count_horizon = int(self.config.time_horizon/self.config.time_slot)
    self.slot_count_day = int((self.config.stop - self.config.start)/self.config.time_slot)
    self.time_to_horizon = self.slot_count_day/self.slot_count_horizon/(self.config.stop - self.config.start).seconds
    self.time_to_slot = self.slot_count_day/(self.config.stop - self.config.start).seconds
    self.schedule_shape = (3 * self.slot_count_horizon)
    self.schedule = np.zeros(self.schedule_shape)
    self.navigator = ROSNavigation()
    
    rospy.wait_for_service('/get_rooms_and_doors')
    self.doors_client = rospy.ServiceProxy('/get_rooms_and_doors', GetRoomsAndDoors)
    
    rospy.wait_for_service('/get_furniture')
    self.furn_client = rospy.ServiceProxy('/get_furniture', GetFurniture)
    
    rospy.wait_for_service('/get_objects')
    self.obj_client = rospy.ServiceProxy('/get_objects', GetObjects)

    rospy.wait_for_service('/perform_pedestrians_step')
    self.env_step_client = rospy.ServiceProxy('/perform_pedestrians_step', Step)
    self.env_step_req = StepRequest()
    self.env_step_req.time = self.config.dt.seconds

    self.robot_publisher = rospy.Publisher('robot', Marker, queue_size=10)
    self.robot_msg = Marker()
    self.robot_msg.header.stamp = rospy.Time.now()
    self.robot_msg.header.frame_id = "map"
    self.robot_msg.lifetime = rospy.Duration.from_sec(0)
    self.robot_msg.id = 0
    self.robot_msg.scale = Vector3(0.5, 0.5, 0.5)
    self.robot_msg.color.a = 1.0
    self.robot_msg.color.r = 1.0
    self.robot_msg.color.g = 0.0
    self.robot_msg.color.b = 0.0
    self.robot_msg.type = 2

    self.reset()

  def close(self):
    if self.config.save:
      try:
        self.estimator_file_out.close()
      except:
        pass

  def reset(self):
    print('Resetting...')

    doors_response = self.doors_client()
    # print(doors_response)
    self.spawn_zones = [((room.x[0] + self.config.robot_radius, room.x[1] - self.config.robot_radius), (room.y[0] + self.config.robot_radius, room.y[1] - self.config.robot_radius)) for room in doors_response.rooms]
    # print(self.spawn_zones)

    furn_response = self.furn_client()
    # print(furn_response)
    self.object_zones = [((room.x[0], room.x[1]), (room.y[0], room.y[1])) for room in furn_response.furniture]
    self.forbidden_spawn_zones = [((room.x[0] - self.config.robot_radius, room.x[1] + self.config.robot_radius), (room.y[0] - self.config.robot_radius, room.y[1] + self.config.robot_radius)) for room in furn_response.furniture]
    # print(self.forbidden_spawn_zones)

    obj_response = self.obj_client()
    # print(obj_response)
    self.objects = [o.id for o in obj_response.objects]

    self.now = self.config.start
    self.tasks = self.task_config.generate(self.spawn_zones, self.forbidden_spawn_zones, self.objects, self.object_zones)

    # initial agent position
    # define absolutes
    x_min = min([zone[0][0] for zone in self.spawn_zones])
    x_max = max([zone[0][1] for zone in self.spawn_zones])
    y_min = min([zone[1][0] for zone in self.spawn_zones])
    y_max = max([zone[1][1] for zone in self.spawn_zones])
    # initialize positions
    x1 = x_min + random.random() * (x_max - x_min)
    y1 = y_min + random.random() * (y_max - y_min)
    # regenerate until proper positions are found
    while(True):
        in_room = False
        for zone in self.spawn_zones:
            if x1 >= zone[0][0] and x1 <= zone[0][1] and y1 >= zone[1][0] and y1 <= zone[1][1]:
                in_room = True
                for fzone in self.forbidden_spawn_zones:
                    if x1 >= fzone[0][0] and x1 <= fzone[0][1] and y1 >= fzone[1][0] and y1 <= fzone[1][1]:
                        in_room = False
                        break
                break
        # if position is inside a room exit loop
        if in_room:
            break
        # generate new posiitons
        x1 = x_min + random.random() * (x_max - x_min)
        y1 = y_min + random.random() * (y_max - y_min)
    self.pos = np.array([x1, y1])

    self.current_job = None
    self.previous_job = None

    # create save files
    if self.config.save:
      now = datetime.now() # current date and time
      fname = self.config.prefix + now.strftime(f"%Y%m%d_%H%M%S_%f")

      fname = 'estimator/timeseries/' + fname + '.csv'
      if not os.path.exists('/'.join(fname.split('/')[:-1])):
        os.makedirs('/'.join(fname.split('/')[:-1]))
      self.estimator_file_out = open(fname, "w")

    # initialize tasker
    self.jobs = []
    self.update_jobs()

    print("Reset done")

  def update_jobs(self):
    # add new jobs
    new_added = False
    self.time_eval = (self.now - self.config.start).seconds*self.time_to_horizon
    for task in self.tasks:
      if not(task in self.jobs):
        if task.calltime <= self.now:
          if task.do_estimate():
            self.jobs.append(task)
            print(f"Added task {task.uuid} to jobs.")
            new_added = True
    # update existing job durations
    if len(self.jobs) and ((self.previous_job != self.current_job and self.previous_job != None) or new_added):
      for job in self.jobs:
        job.updatePos()
        job.distance_from_robot = self.navigator.plan(self.pos, job.pos).get_distance()
        if self.config.use_estimator:
          job.estimated_duration = (timedelta(seconds = np.array(self.estimator(np.expand_dims(np.array([
                      self.time_eval,
                      self.config.day,
                      (job.getDeadline() - self.config.start).seconds*self.time_to_horizon,
                      job.pos[0],
                      job.pos[1],
                      job.goal[0] if isinstance(t, Transport) or isinstance(t, PickAndPlace) else job.pos[0],
                      job.goal[1] if isinstance(t, Transport) or isinstance(t, PickAndPlace) else job.pos[1],
                      self.navigator.plan(self.pos, job.pos).get_distance(),
                      job.getPriority(),
                    ]), axis = 0)))[0,0] * self.config.time_horizon.seconds))
        else:
          job.estimated_duration = job.getBurst() + timedelta(seconds = job.distance_from_robot / self.config.robot_speed)

  def getTaskById(self, task_id):
    return [task for task in self.tasks if task.id == task_id][0]

  def getJobById(self, job_id):
    return [job for job in self.jobs if job.id == job_id][0]

  def removeJobById(self, job_id):
    for i,job in enumerate(self.jobs):
      if job.id == job_id:
        self.jobs.pop(i)
        break

  def execute_step(self, action):
    self.env_step_client(self.env_step_req)

    # select action
    self.previous_job = self.current_job
    self.current_job = action

    # perform action
    if not(self.current_job is None):
      time_left = self.config.dt.seconds

      # do the actual travel to the action spot if distanse from the agent is greater than threshold
      if self.current_job.dist(self.pos) > 0.1:
        path = self.navigator.plan(self.pos, self.current_job.pos)
        [self.pos, time_left] = path.step(self.config.robot_speed, time_left)

      # work on a task
      if time_left > 0:
        self.current_job.do_work(time_left)
        self.pos = self.current_job.pos

      # if task is done remove it from TaskER and job list
      if not self.current_job.do_estimate():
        print('Worked on job ' + str(self.current_job.uuid) + ': ' + str(self.current_job.do_estimate()))
        print('Job ' + str(self.current_job.uuid) + ' complete')
        self.removeJobById(self.current_job.id)
        self.current_job = None
      else:
        print('Worked on job ' + str(self.current_job.uuid) + ': ' + str(self.current_job.do_estimate()))

    self.now = self.now + self.config.dt

    self.robot_msg.pose = Pose(Point(self.pos[0], self.pos[1], 0), Quaternion(0, 0, 0, 1))
    self.robot_publisher.publish(self.robot_msg)

  def save_estimation(self, task):
    self.estimator_file_out.write(':'.join([
        str((self.now - self.config.start).seconds*self.time_to_horizon),
        str(self.config.day),
        str((task.deadline - self.config.start).seconds*self.time_to_horizon),
        str(task.pos[0]),
        str(task.pos[1]),
        str(task.goal[0] if hasattr(task, 'goal') else task.pos[0]),
        str(task.goal[1] if hasattr(task, 'goal') else task.pos[1]),
        str(task.distance_from_robot),
        str(task.priority),
        str(task.estimated_duration.seconds*self.time_to_horizon),
      ]) + '\n')

  def save(self):
    if self.config.save:
      # self.save_schedule()
      for job in self.jobs:
        self.save_estimation(job)
