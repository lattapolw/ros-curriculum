#!/bin/bash

session=$USER

tmux new-session -d -s $session
tmux rename-window -t 0 'Main'
tmux send-key 'roscore' C-m
sleep 3

tmux new-window -t $session:1 -n setupGazebo
tmux send-keys 'cd ../capstone_resources/gazebo/' C-m
tmux send-keys 'export GAZEBO_RESOURCE_PATH=$(pwd)' C-m
tmux send-keys 'rosrun gazebo_ros gazebo world.world' C-m

tmux split-window -h 
tmux send-keys 'cd ../capstone_resources/launch/' C-m
tmux send-keys 'roslaunch sim.launch' C-m

tmux split-window -v
tmux send-keys 'cd ../capstone_resources/launch/' C-m
tmux send-keys 'roslaunch alvar.launch' C-m

tmux select-pane -t 0
tmux split-window -v
tmux send-keys 'cd ../capstone_resources/config/' C-m
tmux send-keys 'rviz -d default.rviz' C-m

tmux attach-session -t $session:1
