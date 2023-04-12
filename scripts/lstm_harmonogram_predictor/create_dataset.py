#!/usr/bin/env python3
from os import listdir
from tensorflow.data import Dataset, experimental
from datetime import datetime

WINDOW_SIZE = 1
SHUFLE_SIZE = 100000

if __name__ == '__main__':
	dirs = ['timeseries/base/', 'timeseries/20%_random_50%_dvar_50%_bvar/']

	timeseries = [dirs[0] + f for f in listdir(dirs[0])]
	for d in dirs[1:]:
		timeseries = timeseries + [d + f for f in listdir(d)]
	timeseries.sort()

	full_dataset = None

	for filename in timeseries:
		print(f'Processing {filename}...')
		file = open(filename, 'r')
		contents = file.read().split('\n')
		file.close()

		while contents[-1] == '':
			contents = contents[:-1]
		contents = [record.split(':') for record in contents]
		contents = [[float(record[0])] + [float(record[1])] + [float(item) for item in record[2][1:-1].split(',')] for record in contents]

		dataset = Dataset.from_tensor_slices(contents)
		dataset = dataset.window(WINDOW_SIZE + 1, shift = 1, drop_remainder = True)
		dataset = dataset.flat_map(lambda window: window.batch(WINDOW_SIZE + 1))
		dataset = dataset.map(lambda window: (window[:-1], window[-1][2:]))

		if not full_dataset:
			full_dataset = dataset
		else:
			full_dataset = full_dataset.concatenate(dataset)

	full_dataset = full_dataset.shuffle(SHUFLE_SIZE, reshuffle_each_iteration = False)
	# for item in full_dataset:
	# 	print(item)
	# 	break
	filename = 'datasets/' + datetime.now().strftime(f"%Y%m%d_%H%M%S_%f_F{len(timeseries)}_W{WINDOW_SIZE}_S{SHUFLE_SIZE}")
	experimental.save(full_dataset, filename)

	# new_dataset = experimental.load(filename)
	# for item in new_dataset:
	# 	print(item)
	# 	break


