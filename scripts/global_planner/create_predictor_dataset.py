#!/usr/bin/env python3
from os import listdir
from tensorflow.data import Dataset, experimental
from datetime import datetime
import sys

WINDOW_SIZE = 2
SHUFLE_SIZE = 1000000

def main(dirs):

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
		contents = [[float(value) for value in record.split(':')] for record in contents]

		dataset = Dataset.from_tensor_slices(contents)
		dataset = dataset.window(WINDOW_SIZE + 1, shift = 1, drop_remainder = True)
		dataset = dataset.flat_map(lambda window: window.batch(WINDOW_SIZE + 1))
		dataset = dataset.map(lambda window: (window[:-1], window[-1][2:]))

		if not full_dataset:
			full_dataset = dataset
		else:
			full_dataset = full_dataset.concatenate(dataset)

	full_dataset = full_dataset.shuffle(SHUFLE_SIZE, reshuffle_each_iteration = False)
	filename = 'predictor/datasets/' + datetime.now().strftime(f"%Y%m%d_%H%M%S_%f_F{len(timeseries)}_W{WINDOW_SIZE}_S{SHUFLE_SIZE}")
	experimental.save(full_dataset, filename)
	with open(filename + '.txt', 'w') as write_file:
		write_file.write('Directories:\n')
		for d in dirs:
			write_file.write(f'{d}\n')
		write_file.write('Timeseries:\n')
		for t in timeseries:
			write_file.write(f'{t}\n')

if __name__ == '__main__':
	if len(sys.argv) < 2:
		print('Pass folders containing files as parameters!')
	else:
		main(sys.argv[1:])
