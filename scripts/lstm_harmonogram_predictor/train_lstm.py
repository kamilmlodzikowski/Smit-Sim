#!/usr/bin/env python3
import tensorflow as tf
from tensorflow.data import Dataset, experimental
import matplotlib.pyplot as plt
from datetime import datetime

BATCH_SIZE = 32
ds_file = 'datasets/20230203_194740_070287_F707_W1_S100000'
epochs = 100
output_folder = 'models/' + datetime.now().strftime(f"%Y%m%d_%H%M%S_%f_E{epochs}_B{BATCH_SIZE}/")

if __name__ == '__main__':
    dataset = experimental.load(ds_file)
    dataset = dataset.batch(BATCH_SIZE).prefetch(1)
    # print(dataset)
    # for item in dataset:
    #     print(item)
    #     break

    model = tf.keras.models.Sequential([ 
        # tf.keras.layers.Lambda(lambda x: tf.expand_dims(x, axis=-1)),
        # tf.keras.layers.Bidirectional(tf.keras.layers.LSTM(1024, return_sequences=True)),
        # tf.keras.layers.Bidirectional(tf.keras.layers.LSTM(512, return_sequences=True)),
        # tf.keras.layers.Bidirectional(tf.keras.layers.LSTM(256, return_sequences=True)),
        # tf.keras.layers.Bidirectional(tf.keras.layers.LSTM(128, return_sequences=True)),
        tf.keras.layers.Bidirectional(tf.keras.layers.LSTM(64)),
        tf.keras.layers.Dense(36),
    ])

    model.compile(loss=tf.keras.losses.Huber(), 
        optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
        metrics=["mae", "mse"]
    )

    history = model.fit(dataset,
        epochs=100,
        callbacks=[tf.keras.callbacks.ModelCheckpoint(output_folder + 'save_{epoch}', save_weights_only = True)],
    )

    # plot MAE and loss
    plt.figure()
    plt.plot(history.history['mae'], label='mae')
    plt.plot(history.history['mse'], label='mse')
    plt.plot(history.history['loss'], label='loss')
    plt.legend()
    plt.savefig(output_folder + 'history.jpg')
    plt.show()