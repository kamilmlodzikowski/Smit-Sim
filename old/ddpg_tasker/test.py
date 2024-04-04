#!/usr/bin/env python3
from my_system import SystemConfig, System
from my_tasks import TaskConfig, TransportGenerator, FallGenerator

if __name__ == '__main__':
	config = SystemConfig()
	task_config = TaskConfig([TransportGenerator, FallGenerator], 5, config.now, config.time_horizon, 100)
	system = System(task_config, config)
