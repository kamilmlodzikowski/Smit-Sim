#!/usr/bin/env python3
from my_system import SystemConfig, System
from my_tasks import TaskConfig, TransportGenerator, FallGenerator, PickAndPlaceGenerator, Transport, Fall, PickAndPlace
from datetime import datetime, date, time
from my_agents import SchedulerAgent, SimpleAgent, DistanceAgent, DQNConfig, DQNAgent, SimpleAgent2
from smit_matlab_sim.srv import FileOperation, FileOperationRequest
from my_eval_functions import StatisticEval, StatisticEvalResult
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
	if not rospy.has_param('~day'):
		sc.day = -1
	else:
		sc.day = int(rospy.get_param('~day'))
	sc.stop = datetime.combine(date.today(), time(12, 0))
	sc.use_estimator = False
	tc = TaskConfig([TransportGenerator, FallGenerator, PickAndPlaceGenerator], 12, sc.start, sc.stop - sc.start, seed = sc.day)
	env = System(tc, sc)
	print(f'Scenario loaded with seed {sc.day}.')

	if not rospy.has_param('~agent_type'):
		rospy.set_param('~agent_type', 'simple')
	agent_type = rospy.get_param('~agent_type')
	if agent_type == 'scheduler':
		agent = SchedulerAgent()
	elif agent_type == 'simple':
		agent = SimpleAgent(float(rospy.get_param('~hesitance')) if rospy.has_param('~hesitance') else 0.0)
	elif agent_type == 'simple2':
		agent = SimpleAgent2(float(rospy.get_param('~hesitance')) if rospy.has_param('~hesitance') else 0.0)
	elif agent_type == 'distance':
		agent = DistanceAgent(float(rospy.get_param('~ratio')) if rospy.has_param('~ratio') else 0.0)
	elif agent_type == 'dqn':
		if not rospy.has_param('~agent_type'):
			print('Pass path to DQN model through dqn_path ros parameter')
		agent_config = DQNConfig()
		agent_config.model_path = rospy.get_param('~dqn_path')
		tasks_per_type = 5
		agent = DQNAgent(agent_config, [Transport, Fall, PickAndPlace], tasks_per_type)
		agent.load()
	else:
		print('Unknown agent type, running SchedulerAgent.')
		agent = SchedulerAgent()

	if agent_type == 'scheduler':
		save_file = f'statistic_eval/{agent_type}/{datetime.now().strftime(f"%Y%m%d_%H%M%S_%f.csv")}'
	elif agent_type == 'simple' or agent_type == 'simple2':
		save_file = f'statistic_eval/{agent_type}_hesitance_{agent.hesitance}/{datetime.now().strftime(f"%Y%m%d_%H%M%S_%f.csv")}'
	elif agent_type == 'distance':
		save_file = f'statistic_eval/{agent_type}_ratio_{agent.ratio}/{datetime.now().strftime(f"%Y%m%d_%H%M%S_%f.csv")}'
	elif agent_type == 'dqn':
		save_file = f'statistic_eval/{agent_type}_{"_".join(agent_config.model_path.split("/")[1:])}/{datetime.now().strftime(f"%Y%m%d_%H%M%S_%f.csv")}'

	eval_fun = StatisticEval(system = env, task_types = [Transport, Fall, PickAndPlace], save_results = True, save_file = save_file)

	result = StatisticEvalResult()

	while(env.now < sc.stop and (not result.terminate)):
		print("Stepping from " + str(env.now) + " to " + str(env.now + sc.dt))
		action = agent.select_task(env.jobs, env.now)
		env.execute_step(action)
		env.update_jobs()
		result = eval_fun.calculate_results(env.tasks, action, env.now)
		# print(result)
		env.save()
	env.close()

