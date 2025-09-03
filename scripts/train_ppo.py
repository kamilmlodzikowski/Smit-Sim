import ray
from ray import tune
from ray.rllib.algorithms.ppo import PPOConfig
from ray.tune.registry import register_env
from ray.tune.integration.wandb import WandbLoggerCallback

from smit_sim_env import SmitSimEnv


def env_creator(env_config):
    return SmitSimEnv()


if __name__ == "__main__":
    ray.init()
    register_env("smit_sim", env_creator)

    config = (
        PPOConfig()
        .environment(env="smit_sim")
        .framework("torch")
        .rollouts(num_rollout_workers=1)
        .training(model={"fcnet_hiddens": [64, 64]})
    )

    tune.run(
        "PPO",
        config=config.to_dict(),
        stop={"training_iteration": 10},
        callbacks=[
            WandbLoggerCallback(project="smit-sim", log_config=True),
        ],
    )
