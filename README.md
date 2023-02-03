# Trajectory Replay
Source ROS
```shell
source /opt/ros/foxy/setup.bash
```
Clone this
```shell
mkdir -p trajectory_replay/ws && cd trajectory_replay/ws && \ 
git clone git@github.com:KCL-BMEIS/trajectory_replay.git && cd ..
```
clone dependencies
```shell
vcs import src < src/trajectory_replay/repos.yml
```
Build
```shell
colcon build
```

## Record Trajectory
Launch the system
```shell
ros2 launch trajectory_recording trajectory_recording.launch.py
```
Start recording
```shell
ros2 bag record /joint_states /link_transform_publisher_node/link_transform
```
Convert trajectory to `.csv` file
```shell
python rosbag_to_csv.py
```

## Play Trajectory
