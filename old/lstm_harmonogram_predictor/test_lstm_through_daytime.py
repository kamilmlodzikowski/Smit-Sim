#!/usr/bin/env python3
import tensorflow as tf
from tensorflow.data import Dataset
import matplotlib.pyplot as plt
import numpy as np
import os

data_filenames = os.listdir('timeseries/base/')
model_filename = 'models/50%_random_50%_dvar_50%_bvar/20230309_123014_432175_E100_B32/save_100'
WINDOW_SIZE = 1

if __name__ == '__main__':
    mse_full = np.zeros(145 - WINDOW_SIZE)
    mae_full = np.zeros(145 - WINDOW_SIZE)
    for data_filename in data_filenames:
        data_file = open('timeseries/base/' + data_filename, 'r')
        contents = data_file.read().split('\n')
        data_file.close()

        while contents[-1] == '':
            contents = contents[:-1]
        contents = [record.split(':') for record in contents]
        contents = [[float(record[0])] + [float(record[1])] + [float(item) for item in record[2][1:-1].split(',')] for record in contents]
        dataset = Dataset.from_tensor_slices(contents)
        dataset = dataset.window(WINDOW_SIZE, shift = 1, drop_remainder = True)
        dataset = dataset.flat_map(lambda window: window.batch(WINDOW_SIZE))
        dataset = dataset.batch(1).prefetch(1)

        model = tf.keras.models.Sequential([
            tf.keras.layers.Bidirectional(tf.keras.layers.LSTM(64)),
            tf.keras.layers.Dense(36),
        ])

        model.compile(loss=tf.keras.losses.Huber(), 
            optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
            metrics=["mae", "mse"]
        )

        model.load_weights(model_filename)

        # forecast = model.predict(dataset)
        forecast = np.array([model(x) for x in dataset]).squeeze()

        mse = tf.keras.metrics.mean_squared_error([x[2:] for x in contents[WINDOW_SIZE:]], forecast[:-1]).numpy()
        mae = tf.keras.metrics.mean_absolute_error([x[2:] for x in contents[WINDOW_SIZE:]], forecast[:-1]).numpy()

        mse_full = mse + mse_full
        mae_full = mae + mae_full

    mse_full = mse_full/len(data_filenames)
    mae_full = mae_full/len(data_filenames)
    print(f'MSE: {mse}\nMAE: {mae}')
    plt.figure()
    plt.plot(mae_full, label='mae')
    plt.plot(mse_full, label='mse')
    plt.legend()
    plt.show()
