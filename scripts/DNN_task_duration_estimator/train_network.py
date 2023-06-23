#!/usr/bin/env python3
import tensorflow as tf
from tensorflow.data import Dataset, experimental
import matplotlib.pyplot as plt
from datetime import datetime

BATCH_SIZE = 32
ds_file = 'datasets/50%_random_50%_dvar_50%_bvar/20230622_141159_780606_F707_S20000000'
epochs = 50
output_folder = 'models/' + datetime.now().strftime(f"%Y%m%d_%H%M%S_%f_E{epochs}_B{BATCH_SIZE}/")

if __name__ == '__main__':
    dataset = experimental.load(ds_file)
    dataset = dataset.batch(BATCH_SIZE).prefetch(1)
    # print(dataset)
    # for item in dataset:
    #     print(item)
    #     break

    model = tf.keras.models.Sequential([ 
#        tf.keras.layers.Dense(256, activation = 'relu'),
#        tf.keras.layers.Dense(128, activation = 'relu'),
        tf.keras.layers.Dense(64, activation = 'relu'),
        tf.keras.layers.Dense(32, activation = 'relu'),
        tf.keras.layers.Dense(1, activation = 'linear'),
    ])

    model.compile(loss=tf.keras.losses.Huber(), 
        optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
        metrics=["mae", "mse"]
    )

    history = model.fit(dataset,
        epochs=epochs,
        callbacks=[tf.keras.callbacks.ModelCheckpoint(output_folder + 'save_{epoch}', save_weights_only = True)],
    )

    # plot MAE and loss
    minimum = min(history.history['loss'])
    plt.figure()
    plt.title(f'Metric values through epochs, lowest loss: {minimum:.5f}')
    plt.xlabel(f'Epoch number')
    plt.ylabel(f'Metric value')
    plt.plot(history.history['mae'], label='mae')
    plt.plot(history.history['mse'], label='mse')
    plt.plot(history.history['loss'], label='loss')
    plt.legend()
    plt.savefig(output_folder + 'history.eps')
    plt.savefig(output_folder + 'history.png')
    plt.show()
