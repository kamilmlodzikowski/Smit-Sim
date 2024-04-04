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

def getAgent(agent_type):
	if agent_type == 'scheduler':
		return SchedulerAgent()
	elif agent_type == 'simple':
		return SimpleAgent(rospy.get_param('~hesitance') if rospy.has_param('~hesitance') else 0.0)
	elif agent_type == 'distance':
		return DistanceAgent(rospy.get_param('~ratio') if rospy.has_param('~ratio') else 0.0)
	else:
		print('Unknown agent type, running SchedulerAgent.')
		return SchedulerAgent()

if __name__ == '__main__':
	rospy.init_node('smit_system')

	rospack = rospkg.RosPack()
	load_map_config_client = rospy.ServiceProxy('/load_config', FileOperation)
	rospy.wait_for_service('/load_config')

	sc = SystemConfig()
	sc.stop = datetime.combine(date.today(), time(12, 0))

	if not rospy.has_param('~agent_type'):
		rospy.set_param('~agent_type', 'scheduler')
	agent_type = rospy.get_param('~agent_type')

	for i in range(7):
		sc.day = i + 1
		sc.prefix = 'base/base_' + str(sc.day) + "_"
		sc.use_estimator = False
		sc.save = True
		tc = TaskConfig([TransportGenerator, FallGenerator, PickAndPlaceGenerator], 12, sc.start, sc.stop - sc.start, seed = sc.day)
		load_map_config_client(FileOperationRequest('/'.join([rospack.get_path('smit_matlab_sim'), 'test_map'])))
		env = System(tc, sc)
		agent = getAgent(agent_type)

		while(env.now < sc.stop):
			print("Stepping from " + str(env.now) + " to " + str(env.now + sc.dt))
			action = agent.select_task(env.jobs, env.now)
			env.execute_step(action)
			env.update_jobs()
			env.save()
		env.close()

		for j in range(50):
			tc.rcount = 1
			tc.d_var = 0.1
			tc.b_var = 0.1
			tc.random_call = False
			sc.prefix = f'{int(100*tc.rcount/tc.count)}%_random_{int(100*tc.d_var)}%_dvar_{int(100*tc.b_var)}%_bvar_random_call_{tc.random_call}/{sc.day}_{j}_'
			load_map_config_client(FileOperationRequest('/'.join([rospack.get_path('smit_matlab_sim'), 'test_map'])))
			env.reset()
			agent = getAgent(agent_type)

			while(env.now < sc.stop):
				print("Stepping from " + str(env.now) + " to " + str(env.now + sc.dt))
				action = agent.select_task(env.jobs, env.now)
				env.execute_step(action)
				env.update_jobs()
				env.save()
			env.close()


	# for i in range(7):
	# 	sc.day = i + 1
	# 	sc.prefix = 'base/base_' + str(sc.day) + "_"
	# 	sc.save = True
	# 	sc.use_estimator = False
	# 	tc = TaskConfig([TransportGenerator, FallGenerator], 60, sc.start, sc.stop - sc.start, seed = sc.day)
	# 	env = System(tc, sc)
	# 	env.run_env()

	# 	for j in range(50):
	# 		tc.rcount = 30
	# 		tc.d_var = 0.1
	# 		tc.b_var = 0.1
	# 		tc.random_call = False
	# 		sc.prefix = f'{int(100*tc.rcount/tc.count)}%_random_{int(100*tc.d_var)}%_dvar_{int(100*tc.b_var)}%_bvar_random_call_{tc.random_call}/{sc.day}_{j}_'
	# 		env.reset()
	# 		env.run_env()
