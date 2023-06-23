#!/usr/bin/env python3
from os import listdir
from tensorflow.data import Dataset, experimental
from datetime import datetime

SHUFLE_SIZE = 20000000

if __name__ == '__main__':
	dirs = ['timeseries/base/', 'timeseries/50%_random_50%_dvar_50%_bvar/']

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
		contents = [record.split(':')[:-1] for record in contents]
		# print(contents[0])
		contents = [[float(item) for item in record] for record in contents]

		dataset = Dataset.from_tensor_slices(contents)
		# dataset = dataset.window(WINDOW_SIZE + 1, shift = 1, drop_remainder = True)
		# dataset = dataset.flat_map(lambda window: window.batch(WINDOW_SIZE + 1))
		dataset = dataset.map(lambda window: (window[:-1], window[-1]))

		if not full_dataset:
			full_dataset = dataset
		else:
			full_dataset = full_dataset.concatenate(dataset)

	full_dataset = full_dataset.shuffle(SHUFLE_SIZE, reshuffle_each_iteration = False)
	# for item in full_dataset:
	# 	print(item)
	# 	break
	filename = 'datasets/' + datetime.now().strftime(f"%Y%m%d_%H%M%S_%f_F{len(timeseries)}_S{SHUFLE_SIZE}")
	experimental.save(full_dataset, filename)

	# new_dataset = experimental.load(filename)
	# for item in new_dataset:
	# 	print(item)
	# 	break


