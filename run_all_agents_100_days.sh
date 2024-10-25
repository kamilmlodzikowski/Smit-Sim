#!/bin/bash

for i in {1..100}
do
	rosrun smit_sim test_planner.py _day:=$i _agent_type:=scheduler
done


for i in {1..100}
do
	rosrun smit_sim test_planner.py _day:=$i _agent_type:=simple _hesitance:=0.0
	rosrun smit_sim test_planner.py _day:=$i _agent_type:=simple _hesitance:=0.25
	rosrun smit_sim test_planner.py _day:=$i _agent_type:=simple _hesitance:=0.5
	rosrun smit_sim test_planner.py _day:=$i _agent_type:=simple _hesitance:=0.75
	rosrun smit_sim test_planner.py _day:=$i _agent_type:=simple _hesitance:=1.0
	rosrun smit_sim test_planner.py _day:=$i _agent_type:=simple2 _hesitance:=0.0
	rosrun smit_sim test_planner.py _day:=$i _agent_type:=simple2 _hesitance:=0.25
	rosrun smit_sim test_planner.py _day:=$i _agent_type:=simple2 _hesitance:=0.5
	rosrun smit_sim test_planner.py _day:=$i _agent_type:=simple2 _hesitance:=0.75
	rosrun smit_sim test_planner.py _day:=$i _agent_type:=simple2 _hesitance:=1.0
	rosrun smit_sim test_planner.py _day:=$i _agent_type:=distance _ratio:=0.0
	rosrun smit_sim test_planner.py _day:=$i _agent_type:=distance _ratio:=0.25
	rosrun smit_sim test_planner.py _day:=$i _agent_type:=distance _ratio:=0.5
	rosrun smit_sim test_planner.py _day:=$i _agent_type:=distance _ratio:=0.75
	rosrun smit_sim test_planner.py _day:=$i _agent_type:=distance _ratio:=1.0
	rosrun smit_sim test_planner.py _day:=$i _agent_type:=dqn _dqn_path:=dqn_agent/20240522_182554_946385/
	rosrun smit_sim test_planner.py _day:=$i _agent_type:=dqn _dqn_path:=dqn_agent/20240617_123007_289512/
	rosrun smit_sim test_planner.py _day:=$i _agent_type:=dqn _dqn_path:=dqn_agent/20240624_144826_202726/
done