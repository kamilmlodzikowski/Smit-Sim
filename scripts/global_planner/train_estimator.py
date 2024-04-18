#!/usr/bin/env python3
import tensorflow as tf
from tensorflow.data import Dataset, experimental
import matplotlib.pyplot as plt
from datetime import datetime
import sys
from contextlib import redirect_stdout
import csv
import os

BATCH_SIZE = 1024
epochs = 50
output_folder = 'estimator/models/' + datetime.now().strftime(f"%Y%m%d_%H%M%S_%f_E{epochs}_B{BATCH_SIZE}")

def get_estimator_model(input_size = 9):
    model = tf.keras.models.Sequential([ 
    tf.keras.layers.Dense(32, input_shape = (input_size,)),
    tf.keras.layers.LeakyReLU(),
    tf.keras.layers.Dense(64),
    tf.keras.layers.LeakyReLU(),
    tf.keras.layers.Dense(128),
    tf.keras.layers.LeakyReLU(),
    tf.keras.layers.Dense(64),
    tf.keras.layers.LeakyReLU(),
    tf.keras.layers.Dense(32),
    tf.keras.layers.LeakyReLU(),
    tf.keras.layers.Dense(1),
    tf.keras.layers.Activation('sigmoid'),
    ])
    assert model.output_shape == (None, 1)
    return model

def main(ds_file):
    # load dataset
    try:
        print(f'Loading dataset: {ds_file}')
        dataset = experimental.load(ds_file)
    except:
        dataset = None
        for dir in os.listdir(ds_file):
            if not os.path.isdir(f'{ds_file}/{dir}'):
                continue
            print(f'Loading dataset: {ds_file}/{dir}')
            if dataset is None:
                dataset = experimental.load(f'{ds_file}/{dir}')
            else:
                dataset = dataset.concatenate(experimental.load(f'{ds_file}/{dir}'))
    dataset = dataset.batch(BATCH_SIZE).prefetch(1)

    # create network model
    model = get_estimator_model()

    # compile model
    model.compile(loss=tf.keras.losses.Huber(),
        optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
        metrics=["mae", "mse"]
    )

    # create model directory
    os.mkdir(output_folder)

    # save name of used dataset
    with open(f'{output_folder}/used_dataset.txt', 'w') as write_file:
        write_file.write(ds_file)

    # save model summary
    with open(f'{output_folder}/model_summary.txt', 'w') as write_file:
        with redirect_stdout(write_file):
            model.summary()

    # train
    try:
        history = model.fit(dataset,
            epochs=epochs,
            callbacks=[tf.keras.callbacks.ModelCheckpoint(output_folder + '/save_{epoch}', save_weights_only = True)],
        )
    except KeyboardInterrupt:
        history = model.history

    # save history
    with open(f'{output_folder}/history.csv', 'w', newline = '') as write_file:
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
    plt.plot(range(1, len(history.history['loss']) + 1), history.history['loss'], label='loss')
    plt.plot(range(1, len(history.history['loss']) + 1), history.history['mae'], label='mae')
    plt.plot(range(1, len(history.history['loss']) + 1), history.history['mse'], label='mse')
    plt.legend()
    plt.savefig(output_folder + '/history.eps')
    plt.savefig(output_folder + '/history.png')
    # plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Pass dataset directory as an argument!')
    else:
        main(sys.argv[1])
