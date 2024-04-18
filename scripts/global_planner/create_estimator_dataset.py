#!/usr/bin/env python3
from os import listdir, mkdir
from tensorflow.data import Dataset, experimental
from datetime import datetime
import sys
import math
import random

SHUFFLE_SIZE = 10000000

def main(dirs):

	# read and shuffle timeseries files' names
	timeseries = [dirs[0] + f for f in listdir(dirs[0])]
	for d in dirs[1:]:
		timeseries = timeseries + [d + f for f in listdir(d)]
	timeseries.sort()
	random.shuffle(timeseries)

	full_dataset = None
	shard_i = 0
	elements = 0
	save_dir = 'estimator/datasets/' + datetime.now().strftime(f"%Y%m%d_%H%M%S_%f_F{len(timeseries)}_S{SHUFFLE_SIZE}")

	# for each file add it to the dataset
	for filename in timeseries:
		print(f'Processing {filename}...', end = '\r')
		file = open(filename, 'r')
		contents = file.read().split('\n')
		file.close()

		# read file and restructure data
		while contents[-1] == '':
			contents = contents[:-1]
		contents = [[float(value) for value in record.split(':')] for record in contents]

		dataset = Dataset.from_tensor_slices(contents)
		dataset = dataset.map(lambda window: (window[:-1], window[-1]))

		# add data to dataset
		if not full_dataset:
			full_dataset = dataset
		else:
			full_dataset = full_dataset.concatenate(dataset)

		# if dataset reaches set size save it and create another
		if full_dataset.cardinality().numpy()/SHUFFLE_SIZE > 1.0:
			print(f'Elements in shard: {full_dataset.cardinality()}')
			elements += full_dataset.cardinality().numpy()
			print(f'Shuffling shard {shard_i}.')
			full_dataset.shuffle(SHUFFLE_SIZE, reshuffle_each_iteration = False)
			print(f'Saving shard {shard_i}.')
			experimental.save(full_dataset, save_dir + f'/{shard_i}')
			shard_i += 1
			full_dataset = None

	# save dataset
	if not (full_dataset is None):
		print(f'Elements in shard: {full_dataset.cardinality()}')
		elements += full_dataset.cardinality().numpy()
		print(f'Shuffling shard {shard_i}.')
		full_dataset.shuffle(SHUFFLE_SIZE, reshuffle_each_iteration = False)
		print(f'Saving shard {shard_i}.')
		experimental.save(full_dataset, save_dir + f'/{shard_i}')
		shard_i += 1
		full_dataset = None

	# save dataset data
	with open(save_dir + '/summary.txt', 'w') as write_file:
		write_file.write(f'Elements in dataset: {elements}\n')
		write_file.write(f'Number of shards: {shard_i}\n')
		write_file.write('Directories:\n')
		for d in dirs:
			write_file.write(f'\t{d}\n')
		write_file.write('Timeseries:\n')
		for t in timeseries:
			write_file.write(f'\t{t}\n')

if __name__ == '__main__':
	if len(sys.argv) < 2:
		print('Pass folders containing files as parameters!')
	else:
		main(sys.argv[1:])
