#!/usr/bin/env python3
from my_system import SystemConfig, System
from my_tasks import TaskConfig, TransportGenerator, FallGenerator

if __name__ == '__main__':
	sc = SystemConfig()
	for i in range(7):
		sc.day = i + 1
		sc.prefix = 'base/base_' + str(sc.day) + "_"
		sc.save = True
		sc.use_estimator = False
		tc = TaskConfig([TransportGenerator, FallGenerator], 60, sc.start, sc.stop - sc.start, seed = sc.day)
		env = System(tc, sc)
		env.run_env()

		for j in range(25):
			tc.rcount = 30
			tc.d_var = 0.5
			tc.b_var = 0.5
			tc.random_call = True
			sc.prefix = f'{int(100*tc.rcount/tc.count)}%_random_{int(100*tc.d_var)}%_dvar_{int(100*tc.b_var)}%_bvar_random_call_{tc.random_call}/{sc.day}_{j}_'
			env.reset()
			env.run_env()
