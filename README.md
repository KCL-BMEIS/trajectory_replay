# Trajectory Replay

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
