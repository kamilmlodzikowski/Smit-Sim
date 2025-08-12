#!/usr/bin/env python3
import gym
from gym import spaces
import rospy
import rospkg
from smit_sim.srv import FileOperation, FileOperationRequest

from my_agents import DQNAgent, DQNConfig


class PPOTrainingSystem(gym.Env):
    """Gym environment wrapper for training with RLlib PPO."""

    def __init__(self, system, eval_function, task_types, tasks_per_type):
        # Initialize ROS node only once; anonymous to avoid name clashes.
        rospy.init_node('smit_system', anonymous=True, disable_signals=True)

        self.rospack = rospkg.RosPack()
        self.load_map_config_client = rospy.ServiceProxy('/load_config', FileOperation)

        self.system = system
        self.eval_function = eval_function
        # Use existing DQNAgent logic only for state representation.
        self.agent = DQNAgent(DQNConfig(), task_types, tasks_per_type)

        self.observation_space = spaces.Box(
            low=0, high=100000, shape=(len(task_types), tasks_per_type, 3)
        )
        self.action_space = spaces.Discrete(len(task_types) * tasks_per_type)

        self.selected_task = None

    def reset(self):
        self.load_map_config_client(
            FileOperationRequest('/'.join([self.rospack.get_path('smit_sim'), 'test_map']))
        )
        self.system.reset()
        self.eval_function.system = self.system
        self.eval_function.reset()
        self.selected_task = None
        self.agent.selected_task = None
        self.agent.calculate_state(self.system.jobs)
        while len(self.system.jobs) == 0:
            self.system.execute_step(self.selected_task)
            self.system.update_jobs()
            self.agent.calculate_state(self.system.jobs)
        return self.agent.state

    def step(self, action):
        if self.selected_task is not None:
            if (not self.selected_task.preemptive) and self.selected_task in self.system.jobs:
                pass
            else:
                self.selected_task = self.agent.tasks_in_state[action]
        else:
            self.selected_task = self.agent.tasks_in_state[action]

        self.system.execute_step(self.selected_task)
        self.system.update_jobs()

        result = self.eval_function.calculate_results(self.selected_task, self.system.now)
        reward = result.reward
        done = result.terminate
        status = "DEAD" if result.dead else (
            "DONE" if result.completed else (
                "TIME" if self.system.now >= self.system.config.stop else "WORK"))
        if status == "TIME":
            done = True

        self.agent.calculate_state(self.system.jobs)
        return self.agent.state, reward, done, {"status": status}

    def render(self):
        pass

    def close(self):
        pass
