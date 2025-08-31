#!/usr/bin/env python3
import csv
import sys
import os
import matplotlib.pyplot as plt
import numpy as np

def main():
	results = {}
	path = sys.argv[1]
	for folder in os.listdir(path):
		if os.path.isdir(f'{path}/{folder}'):
			print(f'Processing {folder}')
			results[folder] = [0 for _ in range(3)]
			for file in os.listdir(f'{path}/{folder}'):
				with open(f'{path}/{folder}/{file}',  newline='') as csvfile:
					csvreader = csv.reader(csvfile, delimiter=';')
					for row in csvreader:
						if row:
							data = row
				data = data[1][1:-1].split(',')
				results[folder] = [results[folder][i] + int(data[i]) for i in range(3)]

	print(results)
	barWidth = 0.25
	fig = plt.subplots()
	h1 = [results[k][0] for k in results.keys()]
	h2 = [results[k][1] for k in results.keys()]
	h3 = [results[k][2] for k in results.keys()]

	br1 = np.arange(len(h1)) 
	br2 = [x + barWidth for x in br1] 
	br3 = [x + barWidth for x in br2] 
 
	# Make the plot
	plt.bar(br1, h1, color ='b', width = barWidth, 
	        edgecolor ='grey', label ='transport') 
	plt.bar(br2, h2, color ='r', width = barWidth, 
	        edgecolor ='grey', label ='fall') 
	plt.bar(br3, h3, color ='g', width = barWidth, 
	        edgecolor ='grey', label ='pickandplace') 
	 
	# Adding Xticks 
	# plt.xlabel('agent type', fontweight ='bold', fontsize = 15) 
	plt.ylabel('number of tasks completed', fontweight ='bold', fontsize = 15) 
	plt.xticks([r + barWidth for r in range(len(h1))], 
	        ['\n-'.join(l.split('-')) for l in list(results.keys())])
	 
	plt.legend()
	plt.show() 

if __name__ == '__main__':
	main()