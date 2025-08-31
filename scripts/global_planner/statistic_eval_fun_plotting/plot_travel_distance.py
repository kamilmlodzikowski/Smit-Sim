#!/usr/bin/env python3
import csv
import sys
import os
import matplotlib.pyplot as plt
import numpy as np

def main():
	file = sys.argv[1]
	category_names1 = ['None'] + [f'transport_{i}' for i in range(12)] + [f'fall_{i}' for i in range(12)] + [f'pickandplace_{i}' for i in range(12)]
	category_names2 = [f'transport_{i}' for i in range(12)]
	category_names3 = [f'fall_{i}' for i in range(12)]
	category_names4 = [f'pickandplace_{i}' for i in range(12)]
	x = []
	y0 = []
	y1 = []
	y2 = []
	y3 = []
	y4 = []
	with open(file,  newline='') as csvfile:
		csvreader = csv.reader(csvfile, delimiter=';')
		i = 0
		data = []
		for line in csvreader:
			x.append(i)
			i += 1
			data.append(line)

		x.pop(0)

		for line in data[1:]:
			if line:
				y0.append(float(line[0]))
				y1.append(category_names1.index(line[7]))
				if line[7] in category_names2:
					y2.append(category_names2.index(line[7]))
				else:
					y2.append(-1)
				if line[7] in category_names3:
					y3.append(category_names3.index(line[7]))
				else:
					y3.append(-1)
				if line[7] in category_names4:
					y4.append(category_names4.index(line[7]))
				else:
					y4.append(-1)

	colors = []

	plt.subplot(1, 1, 1)
	plt.ylabel('UUID')
	plt.axvline(x = x[0])
	plt.axvline(x = x[-1])
	for i,y in enumerate(y1[:-1]):
		if y != y1[i+1]:
			plt.axvline(x = x[i+1])
	for i,y in enumerate(y2[:-1]):
		if y >= 0 and y2[i+1] == y:
			plt.plot(x[i:i+2], [y, y], 'b', label = 'transport')
	for i,y in enumerate(y3[:-1]):
		if y >= 0 and y3[i+1] == y:
			plt.plot(x[i:i+2], [y, y], 'r', label = 'fall')
	for i,y in enumerate(y4[:-1]):
		if y >= 0 and y4[i+1] == y:
			plt.plot(x[i:i+2], [y, y], 'g', label = 'pickandplace')
	plt.legend(bbox_to_anchor=(0, 1),
              loc='lower left', ncol = 3)
	plt.yticks(range(0, 12, 1))
	plt.show()

if __name__ == '__main__':
	main()