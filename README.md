# ped_sim
a ped_sim repo collection of Github

## origin_repo
[onlytailei/gym_ped_sim](https://github.com/onlytailei/gym_ped_sim)

[onlytailei/gym_style_gazebo](https://github.com/onlytailei/gym_style_gazebo)

[srl-freiburg/pedsim_ros](https://github.com/srl-freiburg/pedsim_ros)

## Installation
1. Clone this repo
```shell
mkdir -p ~/ped_sim_ws/src
cd ~/ped_sim_ws/src
git clone https://github.com/Taospirit/ped_sim
```
2. Build package
```shell
cd ../..
catkin_make -j1
catkin_make # may error
catkin_make
```
3. Test roslaunch 
```shell
source ./devel/setup.bash # if use zsh choose to setup.zsh
# test pedsim_ros
roslaunch pedsim_simulator sources_and_sinks.launch
# test gym_ped_sim
roslaunch turtlebot3_social default.launch
# ...some bug to be fixed...

```