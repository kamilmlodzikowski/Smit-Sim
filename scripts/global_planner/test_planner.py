#!/usr/bin/env python3
from my_system import SystemConfig, System
from my_tasks import TaskConfig, TransportGenerator, FallGenerator

if __name__ == '__main__':
	sc = SystemConfig()
	for i in range(1):
		sc.day = i + 1
		sc.save = False
		tc = TaskConfig([TransportGenerator, FallGenerator], 60, sc.start, sc.stop - sc.start, seed = sc.day)
		env = System(tc, sc)
		env.run_env()
		env.close()
