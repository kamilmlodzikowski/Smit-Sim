from __future__ import absolute_import, division, print_function

import numpy as np
import gym
from gym import spaces

from smit_linear_path.linear_path import LinearPath

class SmitSimEnv(gym.Env):
    """Gym wrapper around the existing LinearPath system."""

    metadata = {"render.modes": ["human"]}

    def __init__(self, start=(0.0, 0.0), goal=(1.0, 1.0), dt=0.1, max_speed=1.0):
        super(SmitSimEnv, self).__init__()
        self.start = np.array(start, dtype=np.float32)
        self.goal = np.array(goal, dtype=np.float32)
        self.dt = float(dt)
        self.max_speed = float(max_speed)

        # Actions correspond to forward speed in the range [0, max_speed].
        self.action_space = spaces.Box(
            low=np.array([0.0], dtype=np.float32),
            high=np.array([self.max_speed], dtype=np.float32),
            dtype=np.float32,
        )

        # Observations are the current 2D position of the agent.
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(2,), dtype=np.float32
        )

        self.path = LinearPath(self.start, [self.goal])

    def reset(self):
        """Reset the underlying path and return the starting observation."""
        self.path = LinearPath(self.start, [self.goal])
        return self._get_obs()

    def step(self, action):
        speed = float(np.clip(action[0], 0.0, self.max_speed))
        pos, _ = self.path.step(speed, self.dt)
        obs = pos.astype(np.float32)
        done = self.path.get_distance() == 0
        reward = -np.linalg.norm(self.goal - pos)
        info = {"distance_left": self.path.get_distance()}
        return obs, reward, done, info

    def _get_obs(self):
        return self.path.pos.astype(np.float32)

    def render(self, mode="human"):
        if mode == "human":
            print(
                "pos={}, goal={}, remaining={}".format(
                    self.path.pos, self.goal, self.path.get_distance()
                )
            )
        return None