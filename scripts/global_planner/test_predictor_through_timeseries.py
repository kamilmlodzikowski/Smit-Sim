#!/usr/bin/env python3
import tensorflow as tf
from tensorflow.data import Dataset
import matplotlib.pyplot as plt
import numpy as np
import os
from train_predictor import get_predictor_model
import sys

WINDOW_SIZE = 2

def main(model_filename, dirs):

    model = get_predictor_model(window = WINDOW_SIZE)

    model.compile(loss=tf.keras.losses.Huber(), 
        optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
        metrics=["mae", "mse"]
    )

    model.load_weights(model_filename)

    timeseries = [dirs[0] + f for f in os.listdir(dirs[0])]
    for d in dirs[1:]:
        timeseries = timeseries + [d + f for f in os.listdir(d)]
    timeseries.sort()

    mse_full = np.zeros(145 - WINDOW_SIZE)
    mae_full = np.zeros(145 - WINDOW_SIZE)

    for filename in timeseries:
        print(f'Processing {filename}...')
        file = open(filename, 'r')
        contents = file.read().split('\n')
        file.close()

        while contents[-1] == '':
            contents = contents[:-1]
        contents = [[float(value) for value in record.split(':')] for record in contents]

        dataset = Dataset.from_tensor_slices(contents)
        dataset = dataset.window(WINDOW_SIZE, shift = 1, drop_remainder = True)
        dataset = dataset.flat_map(lambda window: window.batch(WINDOW_SIZE))
        dataset = dataset.batch(1).prefetch(1)

        # forecast = model.predict(dataset)
        forecast = np.array([model(x) for x in dataset]).squeeze()

        mse = tf.keras.metrics.mean_squared_error([x[2:] for x in contents[WINDOW_SIZE:]], forecast[:-1]).numpy()
        mae = tf.keras.metrics.mean_absolute_error([x[2:] for x in contents[WINDOW_SIZE:]], forecast[:-1]).numpy()

        mse_full = mse + mse_full
        mae_full = mae + mae_full

    mse_full = mse_full/len(timeseries)
    mae_full = mae_full/len(timeseries)
    print(f'MSE: {mse}\nMAE: {mae}')
    plt.figure()
    plt.plot(mae_full, label='mae')
    plt.plot(mse_full, label='mse')
    plt.legend()
    plt.show()


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('Pass folders containing files as parameters!')
    else:
        main(sys.argv[1], sys.argv[2:])