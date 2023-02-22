#!/usr/bin/env python3
from os import listdir
from tensorflow.data import Dataset, experimental
from datetime import datetime

WINDOW_SIZE = 1
SHUFLE_SIZE = 100000

if __name__ == '__main__':
	timeseries = listdir('timeseries')
	timeseries.sort()

	full_dataset = None

	for filename in timeseries:
		print(f'Processing {filename}...')
		file = open(f'timeseries/{filename}', 'r')
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

		# for item in dataset:
		# 	print(item)
		# 	break

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


