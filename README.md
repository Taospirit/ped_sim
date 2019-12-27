# ped_sim
a ped_sim repo collection of Github

## origin_repo
[onlytailei/gym_ped_sim](https://github.com/onlytailei/gym_ped_sim)

[onlytailei/gym_style_gazebo](https://github.com/onlytailei/gym_style_gazebo)

[srl-freiburg/pedsim_ros](https://github.com/srl-freiburg/pedsim_ros)

## build
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