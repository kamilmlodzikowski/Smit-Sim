from __future__ import absolute_import, division, print_function

import ray
from ray import tune
from ray.tune.logger import DEFAULT_LOGGERS
from ray.tune.registry import register_env
from ray.tune.integration.wandb import WandbLogger

from smit_sim_env import SmitSimEnv


def env_creator(env_config):
    return SmitSimEnv()


if __name__ == "__main__":
    ray.init()
    register_env("smit_sim", env_creator)

    config = {
        "env": "smit_sim",
        "framework": "torch",
        "num_workers": 1,
        "model": {"fcnet_hiddens": [64, 64]},
        "wandb": {"project": "smit-sim", "log_config": True},
    }

    tune.run(
        "PPO",
        config=config,
        stop={"training_iteration": 10},
        loggers=DEFAULT_LOGGERS + (WandbLogger,),
    )
