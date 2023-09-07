#!/usr/bin/env python3
import tensorflow as tf
from tensorflow.data import Dataset, experimental
import matplotlib.pyplot as plt
from datetime import datetime
import sys
from contextlib import redirect_stdout
import csv

BATCH_SIZE = 32
epochs = 50
output_folder = 'predictor/models/' + datetime.now().strftime(f"%Y%m%d_%H%M%S_%f_E{epochs}_B{BATCH_SIZE}")

def get_predictor_model(window = 2, input_size = 38):
    model = tf.keras.models.Sequential([ 
    tf.keras.layers.Bidirectional(tf.keras.layers.LSTM(64), input_shape = (window, input_size,)),
    tf.keras.layers.LeakyReLU(),
    tf.keras.layers.Dense(input_size - 2, activation = 'linear'),
    ])
    assert model.output_shape == (None, input_size - 2)
    return model

def main(window_size, ds_file):
    # load dataset
    dataset = experimental.load(ds_file)
    dataset = dataset.batch(BATCH_SIZE).prefetch(1)

    # create network model
    model = get_predictor_model(window = window_size)

    # compile model
    model.compile(loss=tf.keras.losses.Huber(), 
        optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
        metrics=["mae", "mse"]
    )

    # train
    history = model.fit(dataset,
        epochs=epochs,
        callbacks=[tf.keras.callbacks.ModelCheckpoint(output_folder + f'_W{window_size}' + '/save_{epoch}', save_weights_only = True)],
    )

    # save name of used dataset
    with open(f'{output_folder}_W{window_size}/used_dataset.txt', 'w') as write_file:
        write_file.write(ds_file)

    # save model summary
    with open(f'{output_folder}_W{window_size}/model_summary.txt', 'w') as write_file:
        with redirect_stdout(write_file):
            model.summary()

    # save history
    with open(f'{output_folder}_W{window_size}/history.csv', 'w', newline = '') as write_file:
        writer = csv.writer(write_file, delimiter = ':')
        metrics = list(history.history)
        writer.writerow(metrics)
        for i in range(len(history.history['loss'])):
            writer.writerow([f'{history.history[m][i]:5f}' for m in metrics])

    # plot MAE and loss
    minimum = min(history.history['loss'])
    min_epoch = history.history['loss'].index(minimum)
    plt.figure()
    plt.title(f'Metric values through epochs, lowest loss: {minimum:.5f} at epoch {min_epoch}')
    plt.xlabel(f'Epoch number')
    plt.ylabel(f'Metric value')
    plt.plot(range(1, epochs + 1), history.history['loss'], label='loss')
    plt.plot(range(1, epochs + 1), history.history['mae'], label='mae')
    plt.plot(range(1, epochs + 1), history.history['mse'], label='mse')
    plt.legend()
    plt.savefig(output_folder + f'_W{window_size}/history.eps')
    plt.savefig(output_folder + f'_W{window_size}/history.png')
    # plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('Pass window size (int) and dataset directory as an argument!')
    else:
        main(int(sys.argv[1]), sys.argv[2])