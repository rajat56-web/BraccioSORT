# BraccioSORT

Video link - https://drive.google.com/drive/folders/1lUViUG80FUB0f_2HXZuZYDuiD9Ch8KWr?usp=drive_link

To Simulate firstly go the Braccio then build and launch  or use
```
cd BraccioSORT
colcon build --symlink-install
source install/setup.bash
ros2 launch braccio_yolo_sorting braccio_moveit_sorting.launch.py
```

## Commands outputs

### ros2 control list_controllers
```
arm_controller          joint_trajectory_controller/JointTrajectoryController  active
gripper_controller      joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster          active
```

### ros2 action list
```
/arm_controller/follow_joint_trajectory 
/execute_trajectory 
/gripper_controller/follow_joint_trajectory 
/move_action 
/sequence_move_group 
```

### ros2 topic hz /camera/image_raw
```
average rate: 21.374 
	min: 0.041s max: 0.059s std dev: 0.00374s window: 23 
average rate: 21.059 
	min: 0.041s max: 0.059s std dev: 0.00380s window: 44 
average rate: 22.387 
	min: 0.035s max: 0.059s std dev: 0.00544s window: 70 
average rate: 23.227 
	min: 0.032s max: 0.059s std dev: 0.00564s window: 96 
average rate: 23.514 
	min: 0.032s max: 0.059s std dev: 0.00532s window: 121 
average rate: 22.667 
	min: 0.032s max: 0.060s std dev: 0.00651s window: 140 
average rate: 21.872 
	min: 0.032s max: 0.064s std dev: 0.00763s window: 158 
average rate: 21.168 
	min: 0.032s max: 0.082s std dev: 0.00893s window: 175 
average rate: 20.493 
	min: 0.032s max: 0.082s std dev: 0.01019s window: 190 
average rate: 19.886 
	min: 0.032s max: 0.082s std dev: 0.01121s window: 205 
average rate: 19.443 
	min: 0.032s max: 0.082s std dev: 0.01158s window: 221 
average rate: 19.073 
	min: 0.032s max: 0.082s std dev: 0.01180s window: 237 
average rate: 18.803
```

### ros2 topic echo /detections
```
header:
  stamp:
    sec: 99
    nanosec: 232000000
  frame_id: camera_optical_link
detections:
- header:
    stamp:
      sec: 99
      nanosec: 232000000
    frame_id: camera_optical_link
  results:
  - hypothesis:
      class_id: red_cube
      score: 0.92
    pose:
      pose:
        position:
          x: 0.0
          y: 0.0
          z: 0.0
        orientation:
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
      covariance:
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
  bbox:
    center:
      position:
        x: 610.0
        y: 158.5
      theta: 0.0
    size_x: 32.0
    size_y: 27.0
  id: ''
```

### ros2 service list | grep compute_ik
```
/compute_ik
```

