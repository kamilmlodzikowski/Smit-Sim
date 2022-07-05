import numpy as np
import rospy
from navfn.srv import MakeNavPlan
import geometry_msgs.msg
from linear_path import LinearPath, Navigation

class ROSNavigation(Navigation):

    def __init__(self):
        rospy.wait_for_service('/planner/make_plan')
        self.planner = rospy.ServiceProxy('/planner/make_plan', MakeNavPlan)

        self.req1 = geometry_msgs.msg.PoseStamped()
        self.req1.pose.orientation.w = 1

        self.req2 = geometry_msgs.msg.PoseStamped()
        self.req2.pose.orientation.w = 1

    def plan(self, start, goal):
        self.req1.pose.position.x = start[0]
        self.req1.pose.position.y = start[1]
        self.req2.pose.position.x = goal[0]
        self.req2.pose.position.y = goal[1]
        resp = self.planner(self.req1, self.req2)
        if resp.plan_found:
            path = [[point.pose.position.x, point.pose.position.y] for point in resp.path]
        else:
            path = [goal]
        return LinearPath(start, path)
