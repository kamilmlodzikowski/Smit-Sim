#!/usr/bin/env python3
from datetime import datetime, date, time

import ray
from ray import air, tune
from ray.rllib.algorithms.ppo import PPOConfig
from ray.tune.registry import register_env
from ray.air.integrations.wandb import WandbLoggerCallback

import rospy

from my_system import SystemConfig, System
from my_tasks import (
    TaskConfig,
    TransportGenerator,
    FallGenerator,
    PickAndPlaceGenerator,
    Transport,
    Fall,
    PickAndPlace,
)
from my_eval_functions import DQNEval
from ppo_training_system import PPOTrainingSystem


def env_creator(env_config):
    """Create a new environment instance for RLlib."""
    rospy.wait_for_service('/load_config')

    sc = SystemConfig()
    sc.stop = datetime.combine(date.today(), time(10, 0))
    sc.use_estimator = False

    tc = TaskConfig(
        [TransportGenerator, FallGenerator, PickAndPlaceGenerator],
        5,
        sc.start,
        sc.stop - sc.start,
        seed=-1,
    )
    tc.instant_call = True

    system = System(tc, sc)
    eval_func = DQNEval(system)
    return PPOTrainingSystem(
        system, eval_func, [Transport, Fall, PickAndPlace], env_config.get("tasks_per_type", 5)
    )


if __name__ == "__main__":
    ray.init()

    register_env("smit_ppo_env", env_creator)

    base_config = (
        PPOConfig()
        .environment(env="smit_ppo_env", env_config={"tasks_per_type": 5})
        .rollouts(num_rollout_workers=0)
    )

    param_space = base_config.to_dict()
    param_space.update(
        {
            "lr": tune.loguniform(1e-4, 1e-2),
            "train_batch_size": tune.grid_search([200, 400]),
        }
    )

    tuner = tune.Tuner(
        "PPO",
        param_space=param_space,
        run_config=air.RunConfig(
            stop={"training_iteration": 10},
            callbacks=[
                WandbLoggerCallback(project="smit_sim", log_config=True)
            ],
        ),
    )
    tuner.fit()
