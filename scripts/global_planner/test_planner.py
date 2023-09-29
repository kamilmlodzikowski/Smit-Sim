#!/usr/bin/env python3
from my_system import SystemConfig, System
from my_tasks import TaskConfig, TransportGenerator, FallGenerator

if __name__ == '__main__':
	sc = SystemConfig()
	sc.day = 1
	sc.estimator_path = 'estimator_horizon_based/models/20230919_193255_013892_E50_B32/save_50'
	sc.use_estimator = False
	tc = TaskConfig([TransportGenerator, FallGenerator], 60, sc.start, sc.stop - sc.start, seed = sc.day)
	env = System(tc, sc)
	env.run_env()
