# Trajectory Replay
Source ROS
```shell
source /opt/ros/humble/setup.bash
```
Clone this
```shell
mkdir -p trajectory_replay_ws/src && cd trajectory_replay_ws/src && \
git clone https://github.com/KCL-BMEIS/trajectory_replay.git && cd ..
```
Clone dependencies
```shell
vcs import src < src/trajectory_replay/repos.yml
```
Build
```shell
colcon build && \
source install/setup.bash
```

## Record Trajectory
- Launch the system
```shell
ros2 launch lbr_demos_fri_ros2_advanced_cpp admittance_control_node.launch.py model:=med7 # port_id:=[30200, 30201] (depends on configuration)
```
- Start recording
```shell
ros2 bag record /lbr/state
```
- Convert trajectory to `.csv` file
```shell
python rosbag_to_csv.py --rosbag_path <path_to_rosbag> --lbr_state_topic /lbr/state
```

## Play Trajectory
- Turn on robot
- Remove everything with cables from the end-effector
- Execute braketest, robot will move upright! (AUT ot T1 mode)
- Put robot into T1 mode
- Move robot into desired start position (close to brain)
- Launch `LBRServer`. (10ms -> 172.31.1.148 -> POSITION_CONTROL -> POSITION)
- Open terminal and run
```shell
source trajectory_replay/install/setup.bash
ros2 launch lbr_bringup bringup.launch.py model:=med7 sim:=false # port_id:=[30200, 30201] (depends on configuration)
```
- Put robot into AUT mode
- Open another terminal and run
```shell
source trajectory_replay/install/setup.bash
ros2 launch trajectory_replay joint_trajectory_client_node.launch.py seconds_from_start:=1 # integer >= 1
```
