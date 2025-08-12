# Smit Matlab Sim

## Instalation

1. Install appropriate libraries.
- Install ROS melodic bo following instructions from their site. For native installation on Ubuntu 18 see [http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu).
- Install AI libraries (tensorflow, keras-rl, gym).
```
pip3 install tensorflow
pip3 install keras-rl
pip3 install gym
pip3 install "ray[rllib]"
pip3 install wandb
```
- Install the [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python). It can be installed using pip.
```
pip3 install roboticstoolbox-python
```
As this library links to the others created by the same developer, if any exceptions are thrown during execution of random_map_server.py script (see **Running** section), the offending pip package should be removed and the same package should be installed directly from the [sources](https://github.com/petercorke) to get the newest version.

2. Create workspace.
```
mkdir -p smit_ws/src
cd smit_ws/src/
```
3. Clone ROS packages into the workspace. YOu need to download all three of them for the software to work.
- smit_sim (this package).
```
git clone https://github.com/RCPRG-ros-pkg/Smit-Sim.git
```
- [tasker](https://github.com/RCPRG-ros-pkg/tasker/tree/smit-reqTab) on smit-reqTab branch.
```
git clone -b smit-reqTab https://github.com/RCPRG-ros-pkg/tasker.git
```
- [tasker_msgs](https://github.com/RCPRG-ros-pkg/tasker_msgs/tree/smit) on smit branch
```
git clone -b smit https://github.com/RCPRG-ros-pkg/tasker_msgs.git
```
4. Build the workspace.
```
cd ../..
catkin build
```

## Important files
1. scripts/random_map_server.py - runs the environment - map with obstacles and pedestrians.
2. scripts/global_planner/my_tasks.py - tasks used in our system.
3. scripts/global_planner/my_system.py - scenario that runs tasks.
4. scripts/global_planner/my_agents.py - agents that decide which task to perform.
5. scripts/global_planner/my_eval_functions.py - evaluation functions for assessing agents' work.
6. test_map - default environment configuration.

## Running the environment and utilizing local planner
The system utilizes ROS framework using Python3 and was tested on Ubuntu 18.04.

### Running

1. Build your workspace, including this package, using ROS (catkin build) - see last step of **Installation**.
2. In first console (starting from ROS workspace) run the local planner. Doing this before running the map script also starts roscore.
```
source devel/setup.bash
roslaunch smit_matlab_sim tasks_execution.launch
```
3. In second console (starting from ROS workspace) run the map server. This script can be run with many different arguments that change the map's outcome. This are not ROS arguments, but are implemented using argparse library instead. See the bottom of the script's contents for the argument list.
```
source devel/setup.bash
rosrun smit_matlab_sim random_map_server.py
```
4. (Optional) Load previously created map by sending its filename by a ROS service called /load_config. Service type is FileOperation (included in this package). You can do it with rqt or by console command (example below). Config file directory should be passed in regard to the random_map_server.py file. Sample config file is present in this package under the name 'test_map'.
```
source devel/setup.bash
rosservice call /load_config "filename: 'config_file_directory/config_file_name'"
```
5. (Optional) Run rviz in separate console to see the map with objects on it. The sample configuration is in this package under the name 'smit_new.rviz'.
```
source devel/setup.bash
rosrun rviz rviz
```

### Planning route

To get a route planned according to used map you can call service /planner/make_plan of type MakeNavPlan (from navfn package). Alternatively you can utilize class ROSNavigation from this package to get object of class LinearPath. You can run LinearPath.step(robot_speed, move_duration_in_secs) function to move through the path with constant speed for set amount of time.

### Adding pedestrians

You can use /add_pedestrian service of type AddPedestrian to add pedestrians to map. It has the following components:
1. velocity - float64 - pedestrian velocity during move
2. path - std_msgs/Float64MultiArray - list of points from start to goal, if empty generated randomly
3. full_path - bool - informs if the path is fully planned or only consists of start and goal points
4. behavior - int8 - pedestrian behavior, 1 - go around in circles, 2 - plan new random path after completing previous one, 3 - dissapear after path completion, 4 - plan new random path that starts in stop place after of previous one

## Running the scenario for robot performing tasks
The system utilizes ROS framework using Python3 and was tested on Ubuntu 18.04.

1. Perform steps 1-3 from the **Running and utilizing random_map_server with local planner** - **Running** section above.
2. In third console run test_planner.py script (can be found in scripts/global_planner directory). Currently, this script always loads the 'test_map' map configuration found in this package.
```
source devel/setup.bash
rosrun smit_matlab_sim test_planner.py _agent_type:=distance _ratio:=1.0
```
The script can be configured using ROS parameters passed by console. For example passing a day parameter will determine the random seed for task generation. Example for day 1 below.
```
rosrun smit_matlab_sim test_planner.py _day:=1
```
The script uses the Simple agent by default. One can change the agent type using the parameters. Examples below. Most agent types also requires additional configuration parameter. 'ratio' and 'hesitance' should be a float number between 0 and 1, while 'dqn_path' should be a path to the network's directory. Examples are presented below.
```
rosrun smit_matlab_sim test_planner.py _agent_type:=distance _ratio:=0.5
rosrun smit_matlab_sim test_planner.py _agent_type:=simple _hesitance:=0.5
rosrun smit_matlab_sim test_planner.py _agent_type:=simple2 _hesitance:=0.5
rosrun smit_matlab_sim test_planner.py _agent_type:=scheduler
rosrun smit_matlab_sim test_planner.py _agent_type:=dqn dqn_path:=<path_to_network>
```

## Training a DQNAgent
The system utilizes ROS framework using Python3 and was tested on Ubuntu 18.04.

1. Perform steps 1-3 from the **Running and utilizing random_map_server with local planner** - **Running** section above.
2. In third console run train_dqnagent.py script (can be found in scripts/global_planner directory). Currently, this script always loads the 'test_map' map configuration found in this package. This script trains the DQNAgent, whose code can be found found in scripts/global_planner/my_agents.py file. It uses DQNEval reward function from scripts/global_planner/my_eval_functions.py.
```
source devel/setup.bash
rosrun smit_matlab_sim train_dqnagent.py
```

## Training a PPO agent with RLlib
The system utilizes ROS framework using Python3 and was tested on Ubuntu 18.04.

1. Perform steps 1-3 from the **Running and utilizing random_map_server with local planner** - **Running** section above.
2. In third console run train_ppo_rllib.py script (located in scripts/global_planner directory). This script uses Ray Tune for hyperparameter search and logs metrics to Weights & Biases.
```
source devel/setup.bash
rosrun smit_matlab_sim train_ppo_rllib.py
```
