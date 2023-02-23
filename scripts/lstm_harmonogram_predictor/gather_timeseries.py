#!/usr/bin/env python3
from my_system import SystemConfig, System
from my_tasks import TaskConfig, TransportGenerator, FallGenerator

if __name__ == '__main__':
	sc = SystemConfig()
	for i in range(7):
		sc.day = i + 1
		sc.prefix = 'base_' + str(sc.day) + "_"
		sc.save = False
		tc = TaskConfig([TransportGenerator, FallGenerator], 60, sc.start, sc.stop - sc.start, seed = sc.day)
		env = System(tc, sc)
		# env.run_env()
		# env.close()

		for j in range(100):
			sc.save = True
			sc.prefix = str(sc.day) + "_" + str(j) + "_"
			tc.rcount = 30
			env.reset()
			env.run_env()
			env.close()
