#!/usr/bin/env python3
import csv
import sys
import os
import matplotlib.pyplot as plt
import numpy as np

def survey(results, category_names):
	labels = ['\n-'.join(k.split('-')) for k in list(results.keys())]
	data = np.array(list(results.values()))
	data_cum = data.cumsum(axis=1)
	category_colors = plt.get_cmap('RdYlGn')(
		np.linspace(0.15, 0.85, data.shape[1]))

	fig, ax = plt.subplots(figsize=(9.2, 5))
	ax.invert_yaxis()
	ax.xaxis.set_visible(False)
	ax.set_xlim(0, np.sum(data, axis=1).max())

	for i, (colname, color) in enumerate(zip(category_names, category_colors)):
		widths = data[:, i]
		starts = data_cum[:, i] - widths
		ax.barh(labels, widths, left=starts, height=0.5,
				label=colname, color=color)
		xcenters = starts + widths / 2

		r, g, b, _ = color
		text_color = 'white' if r * g * b < 0.5 else 'darkgrey'
	plt.subplots_adjust(right = 0.99)
	ax.legend(ncol=len(category_names), bbox_to_anchor=(0, 0),
			  loc='upper left')

	return fig, ax

def main():
	plt.rcParams['font.size'] = 16
	category_names = ['complete', 'dead', 'oscilation', 'time']
	results = {}
	path = sys.argv[1]
	for folder in os.listdir(path):
		if os.path.isdir(f'{path}/{folder}'):
			print(f'Processing {folder}')
			results[folder] = [0 for _ in range(4)]
			for file in os.listdir(f'{path}/{folder}'):
				with open(f'{path}/{folder}/{file}',  newline='') as csvfile:
					csvreader = csv.reader(csvfile, delimiter=';')
					for row in csvreader:
						if row:
							data = row
				if data[-3] == 'True':
					results[folder][0] += 1
				elif data[-2] == 'True':
					results[folder][1] += 1
				elif data[-1] == 'True':
					results[folder][2] += 1
				else:
					results[folder][3] += 1

	print(results)
	survey(results, category_names)
	plt.title('Termination statistics per agent used')
	plt.show()

if __name__ == '__main__':
	main()