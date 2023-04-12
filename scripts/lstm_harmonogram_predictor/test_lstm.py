#!/usr/bin/env python3
import tensorflow as tf
from tensorflow.data import Dataset
import matplotlib.pyplot as plt
import numpy as np

data_filename = 'timeseries/50%_random_fixed/1_0_20230301_172620_826569_1.csv'
model_filename = 'models/50%_random_fixed/20230302_114234_896991_E100_B32/save_100'
WINDOW_SIZE = 1

if __name__ == '__main__':
    data_file = open(data_filename, 'r')
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
    print(forecast[0])
    print(contents[WINDOW_SIZE][2:])

    plt.figure()
    plt.scatter(range(36), forecast[0], label='forecast')
    plt.scatter(range(36), contents[WINDOW_SIZE][2:], label = 'original')
    plt.legend()
    plt.show()

    mse = tf.keras.metrics.mean_squared_error([x[2:] for x in contents[WINDOW_SIZE:]], forecast[:-1]).numpy()
    mae = tf.keras.metrics.mean_absolute_error([x[2:] for x in contents[WINDOW_SIZE:]], forecast[:-1]).numpy()

    print(f'MSE: {mse}\nMAE: {mae}')
    plt.figure()
    plt.plot(mae, label='mae')
    plt.plot(mse, label='mse')
    plt.legend()
    plt.show()
