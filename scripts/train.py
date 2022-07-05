#!/usr/bin/env python3
import numpy as np
import random
from tensorflow.keras.layers import Dense, Flatten
from tensorflow.keras.optimizers import Adam
import tensorflow as tf
from rl.agents.dqn import DQNAgent
from rl.policy import EpsGreedyQPolicy, LinearAnnealedPolicy
from rl.memory import SequentialMemory
from task import TransportGenerator, FallGenerator, DriveGenerator
from tasks_env import Tasks

def build_model(states, actions):
    model = tf.keras.Sequential()
    model.add(Flatten(input_shape=(1,)+ states))
    model.add(Dense(50, activation='relu'))
    model.add(Dense(actions, activation='softmax'))
    model.add(Flatten())
    return model

def buildAgent(model, actions):
    policy = LinearAnnealedPolicy(EpsGreedyQPolicy(), 'eps', 1.0, 0.1, 0.1, 30000)
    memory = SequentialMemory(limit=5000, window_length=1)
    dqn = DQNAgent(model, memory=memory, policy=policy, nb_actions=actions, nb_steps_warmup=5000,
                   target_model_update=1e-2)
    return dqn

if __name__ == '__main__':
	task_desc = [
	    DriveGenerator,
	    FallGenerator
	]

	# penalty = 2
	N = 5
	substeps = 1

	seed = 144
	random.seed(seed)
	system = Tasks(task_desc, N, substeps, False)

	states = system.state_space.shape
	actions = system.action_space.n

	model = build_model(states, actions)
	model.summary()

	DQN = buildAgent(model, actions)

    # train
	DQN.compile(tf.keras.optimizers.Adam(learning_rate=1e-3), metrics=['mae'])
	DQN.fit(system, nb_steps=30000, visualize=False, verbose=2)

    # test
	system = Tasks(task_desc, N, substeps)
	scores = DQN.test(system, nb_episodes=100, visualize=False)
	print(np.mean(scores.history['episode_reward']))

    # run
	system = Tasks(task_desc, N, substeps)
	seed = random.randint(0, 10000)
	random.seed(seed)
	system.reset()

	state = system.state
	states = state.flatten()
	states = np.expand_dims(states, -1)
	tasks = []
	rewards = []
	outs = []

	#legend = system.legend()
	#legend = [ f'{i} | {l}' for i,l in enumerate(legend)]

	t = 0
	# system.render()
	done = False
	while not done:
	  preds = model.predict([[np.expand_dims(state, 0)]])
	  outs.append(preds[0])
	  action = np.argmax(preds[0])
	  tasks.append(action)
	  
	  state, reward, done, info = system.step(action)

	  
	  states = np.append(states, np.expand_dims(state.flatten(), -1), -1)
	  rewards.append(reward)

	# system.render()
	print(states.shape)

	from matplotlib import pyplot as plt
	plt.figure(figsize=(15,10)).tight_layout()
	plt.subplot(3,1,1)
	plt.plot(np.transpose(states))
	#plt.legend(legend)
	ax = plt.subplot(3,1,2)
	ax.imshow(np.transpose(outs), aspect='auto')
	# plt.plot(tasks, "o")
	ax = plt.subplot(3,1,3)
	plt.plot(rewards, "o")
	plt.suptitle('seed: ' + str(seed) + ' | dt: ' + str(system.dt), fontsize=16)
	plt.show()
