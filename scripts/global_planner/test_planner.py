#!/usr/bin/env python3
from my_system import SystemConfig, System
from my_tasks import TaskConfig, TransportGenerator, FallGenerator, PickAndPlaceGenerator
from datetime import datetime, date, time
from my_agents import SchedulerAgent, SimpleAgent, DistanceAgent
from smit_matlab_sim.srv import FileOperation, FileOperationRequest
import random
import rospy
import sys
import rospkg

if __name__ == '__main__':
	rospy.init_node('smit_system')

	rospack = rospkg.RosPack()
	load_map_config_client = rospy.ServiceProxy('/load_config', FileOperation)
	rospy.wait_for_service('/load_config')
	load_map_config_client(FileOperationRequest('/'.join([rospack.get_path('smit_matlab_sim'), 'test_map'])))

	sc = SystemConfig()
	sc.day = 1
	sc.stop = datetime.combine(date.today(), time(12, 0))
	sc.use_estimator = False
	tc = TaskConfig([TransportGenerator, FallGenerator, PickAndPlaceGenerator], 12, sc.start, sc.stop - sc.start, seed = sc.day)
	env = System(tc, sc)

	if not rospy.has_param('~agent_type'):
		rospy.set_param('~agent_type', 'scheduler')
	agent_type = rospy.get_param('~agent_type')
	if agent_type == 'scheduler':
		agent = SchedulerAgent()
	elif agent_type == 'simple':
		agent = SimpleAgent(rospy.get_param('~hesitance') if rospy.has_param('~hesitance') else 0.0)
	elif agent_type == 'distance':
		agent = DistanceAgent(rospy.get_param('~ratio') if rospy.has_param('~ratio') else 0.0)
	else:
		print('Unknown agent type, running SchedulerAgent.')
		agent = SchedulerAgent()

	while(env.now < sc.stop):
		print("Stepping from " + str(env.now) + " to " + str(env.now + sc.dt))
		action = agent.select_task(env.jobs, env.now)
		env.execute_step(action)
		env.update_jobs()
		env.save()
	env.close()

