# SUSTech EE368 Robotic Motion and Control Project

## Development
### Software Architecture

Framework: ROS1 Neotic (Ubuntu 20.04)

#### packages:
- gomoku_vision
- gomoku_ai
- gomoku_control

#### Messages:
- 'arm_pose' geometry_msgs/Pose

    from gomoku_vision to gomoku_control

- 'piece_position' gomoku_vision/Position

    from gomoku_vision to gomoku_ai

- 'next_move' gomoku_vision/Position

    from gomoku_ai to gomoku_vision

#### Run:
- $ roscore
- $ roslaunch kortex_driver kortex_driver.launch
- $ roslaunch gomoku_control pick_and_place_example.launch
- Test example: $ rostopic pub /my_gen3_lite/arm_point geometry_msgs/Point '{x: 0.427, y: 0.07, z: 0.1}'

### Hardware Architecture
- Kinova Gen3 Lite: 6DOF Robot Arm
- Intel RealSense D435: RGBD Camera
- Lego 2x2 bricks: Gomoku pieces

### Commits
Follow the format proposed in the [Semantic Commits](https://gist.github.com/joshbuchea/6f47e86d2510bce28f8e7f42ae84c716)
