import numpy as np

class LinearPath:
  def __init__(self, start, points):
    self.points = np.array(points)
    self.pos = np.array(start)

  def step(self, spd, dt):
    time_left = dt


    while time_left > 0:
      # last point reached
      if self.points.shape[0] < 1:
        break

      dist = np.linalg.norm(self.points[0,:] - self.pos)
      eta = dist / spd
      # time to reach next point shorter than allowed movement time
      if eta < time_left:
        self.pos = self.points[0,:]
        self.points = np.delete(self.points, 0, 0)
        time_left = time_left - eta
      # next point is farther than remaining time
      else:
        v = (self.points[0,:] - self.pos) / dist
        self.pos = self.pos + v * spd * time_left
        time_left = 0

    return (self.pos, time_left)

  def get_distance(self):
    if self.points.shape[0] < 1:
      return 0
    return sum([np.linalg.norm(self.points[0] - self.pos)] + [np.linalg.norm(self.points[i+1] - self.points[i]) for i in range(len(self.points)-1)])

  def print(self):
    print(self.pos, self.points)

class Navigation:
  def plan(self, start, goal):
    raise NotImplementedError()

class LinearNav(Navigation):
  def plan(self, start, goal):
    return LinearPath(start, [goal])