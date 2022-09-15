#!/usr/bin/env python3
import numpy as np
import random
import os
import sys
sys.path.insert(0, '../../../dqn')

from dqn_agent import *
from system import SystemConfig
from my_system import MySystem
from tasks import TaskConfig, FallGenerator
from my_tasks import DriveGenerator

from datetime import datetime

def timestr(prefix = "", suffix = ""):
    now = datetime.now() # current date and time
    return prefix + now.strftime("%Y%m%d_%H%M%S_%f") + suffix

def objpickle(obj, folder, fname):
    import jsonpickle
    with open(os.path.join(folder, fname), "w") as f:
        f.write(jsonpickle.encode(obj, indent=4))

out_folder = timestr("testy/", "")
if not os.path.exists(out_folder):
    os.makedirs(out_folder)
else:
    print("WARN: output folder exists, logs may be mixed")

config = SystemConfig()
config.robot_speed = 0.1
config.save = True
config.prefix = out_folder + "/test_"

dqn_config = DQNConfig()
dqn_config.training_steps = 10000

seed = random.randint(0, 10000)
task_config = TaskConfig([DriveGenerator, FallGenerator], 5, seed, 0.3)

system = MySystem(task_config, config)

import jsonpickle
out_folder = timestr("model_")
os.makedirs(out_folder)
objpickle(config, out_folder, 'config_system.json')
objpickle(task_config, out_folder, 'config_task.json')
objpickle(dqn_config, out_folder, 'config_dqn.json')

model = dqn_train(system, dqn_config)
model.save(out_folder)
print(dqn_config.model)

print(task_config.generate())

system.reset()
ret = run_dqn_agent(system, model)
system.close()
plot_dqn_agent(*ret, seed)
plt.show()