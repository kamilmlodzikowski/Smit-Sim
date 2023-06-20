#!/usr/bin/env python3
from my_system import SystemConfig, System
from my_tasks import TaskConfig, TransportGenerator, FallGenerator

if __name__ == '__main__':
	sc = SystemConfig()
	for i in range(7):
		sc.day = i + 1
		sc.prefix = 'base/base_' + str(sc.day) + "_"
		sc.save = False
		tc = TaskConfig([TransportGenerator, FallGenerator], 60, sc.start, sc.stop - sc.start, seed = sc.day)
		env = System(tc, sc)
		# env.run_env()
		env.close()

		for j in range(100):
			sc.save = True
			tc.rcount = 30
			tc.d_var = 0.5
			tc.b_var = 0.5
			sc.prefix = f'{int(100*tc.rcount/tc.count)}%_random_{int(100*tc.d_var)}%_dvar_{int(100*tc.b_var)}%_bvar/{sc.day}_{j}_'
			env.reset()
			env.run_env()
			env.close()
