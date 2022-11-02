#!/usr/bin/env python3
import numpy as np
import gym
import os

from my_system import SystemConfig, System
from my_tasks import TaskConfig, TransportGenerator, FallGenerator

from tensorflow.keras.models import Sequential, Model
from tensorflow.keras.layers import Dense, Activation, Flatten, Input, Concatenate
from tensorflow.keras.optimizers import Adam

from rl.agents import DDPGAgent
from rl.memory import SequentialMemory
from rl.random import OrnsteinUhlenbeckProcess

from datetime import datetime

if __name__ == '__main__':
	config = SystemConfig()
	task_config = TaskConfig([TransportGenerator, FallGenerator], 5, config.now, config.time_horizon, 100)
	env = System(task_config, config)

	nb_actions = env.action_space.shape[0]

	# Next, we build a very simple model.
	actor = Sequential()
	actor.add(Flatten(input_shape=(1,) + env.state_space.shape))
	# actor.add(Dense(16))
	# actor.add(Activation('relu'))
	actor.add(Dense(16))
	actor.add(Activation('relu'))
	actor.add(Dense(16))
	actor.add(Activation('relu'))
	actor.add(Dense(nb_actions))
	actor.add(Activation('sigmoid'))
	print(actor.summary())

	action_input = Input(shape=(nb_actions,), name='action_input')
	observation_input = Input(shape=(1,) + env.state_space.shape, name='observation_input')
	flattened_observation = Flatten()(observation_input)
	x = Concatenate()([action_input, flattened_observation])
	# x = Dense(32)(x)
	# x = Activation('relu')(x)
	x = Dense(32)(x)
	x = Activation('relu')(x)
	x = Dense(32)(x)
	x = Activation('relu')(x)
	x = Dense(1)(x)
	x = Activation('linear')(x)
	critic = Model(inputs=[action_input, observation_input], outputs=x)
	print(critic.summary())

	# from tensorflow import keras
	# actor = keras.models.load_model('tests/ddpg_weights_recent_actor.h5f')
	# critic = keras.models.load_model('tests/ddpg_weights_recent_critic.h5f')


	# Finally, we configure and compile our agent. You can use every built-in tensorflow.keras optimizer and
	# even the metrics!
	memory = SequentialMemory(limit=5000, window_length=1)
	random_process = OrnsteinUhlenbeckProcess(size=nb_actions, theta=0.15, mu=0., sigma=0.1)
	agent = DDPGAgent(nb_actions=nb_actions, actor=actor, critic=critic, critic_action_input=action_input,
	                  memory=memory, nb_steps_warmup_critic=20, nb_steps_warmup_actor=20,
	                  random_process=random_process, gamma=.99, target_model_update=1e-3)
	agent.compile(Adam(learning_rate=.001, clipnorm=1.), metrics=['mae'])
	agent.load_weights(f'tests/ddpg_weights_recent.h5f')

	# Okay, now it's time to learn something! We visualize the training here for show, but this
	# slows down training quite a lot. You can always safely abort the training prematurely using
	# Ctrl + C.
	agent.fit(env, nb_steps=10000, visualize=False, verbose=1, nb_max_episode_steps=2500)

	# After training is done, we save the final weights.
	agent.save_weights(f'tests/ddpg_weights_{datetime.now()}.h5f', overwrite=True)
	agent.save_weights(f'tests/ddpg_weights_recent.h5f', overwrite=True)

	# Finally, evaluate our algorithm for 1 episodes.
	# agent.test(env, nb_episodes=1, visualize=False, nb_max_episode_steps=10000)