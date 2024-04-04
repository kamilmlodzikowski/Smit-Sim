#!/usr/bin/env python3
from my_system import SystemConfig, System
from my_tasks import TaskConfig, TransportGenerator, FallGenerator, Transport, Fall
import csv

if __name__ == '__main__':
	sc = SystemConfig()
	for i in range(7):
		sc.day = i + 1
		sc.prefix = 'base/base_' + str(sc.day) + "_"
		sc.save = False
		tc = TaskConfig([TransportGenerator, FallGenerator], 60, sc.start, sc.stop - sc.start, seed = sc.day)
		env = System(tc, sc)
		# env.run_env()
		# env.close()

		t_file = open(f'saved_tasks/transport_day{i+1}.csv', 'w', newline='')
		f_file = open(f'saved_tasks/fall_day{i+1}.csv', 'w', newline='')

		t_writer = csv.DictWriter(t_file, fieldnames = ['id', 'uuid', 'priority', 'pos', 'start_time', 'deadline', 'goal', 'spd', 'path'])
		f_writer = csv.DictWriter(f_file, fieldnames = ['id', 'uuid', 'priority', 'pos', 'start_time', 'deadline', 'urgency'])
		t_writer.writeheader()
		f_writer.writeheader()

		for task in env.tasks:
			if isinstance(task, Transport):
				t_writer.writerow({'id': task.id,
					'uuid': task.uuid,
					'priority':task.priority,
					'pos': task.pos,
					'start_time': (task.getDeadline() - task.getBurst()).strftime(f"%H:%M:%S"),
					'deadline': task.getDeadline().strftime(f"%H:%M:%S"),
					'goal': task.goal,
					'spd': task.spd,
					'path': ','.join([str(p) for p in task.path.points[:-1]]),
				})
			elif isinstance(task, Fall):
				f_writer.writerow({'id': task.id,
					'uuid': task.uuid,
					'priority':task.priority,
					'pos': task.pos,
					'start_time': (task.getDeadline() - task.getBurst()).strftime(f"%H:%M:%S"),
					'deadline': task.getDeadline().strftime(f"%H:%M:%S"),
					'urgency': 3600 - (task.getDeathTime() - (task.getDeadline() - task.getBurst())).seconds,
				})

		t_file.close()
		f_file.close()