# SUSTech EE368 Robotic Motion and Control Project

## Development
### Software Architecture

Framework: ROS1 Neotic (Ubuntu 20.04)

Three packages:
- gomoku_vision
- gomoku_ai
- gomoku_control

Message Type:
- gomoku_vision/Position
Between gomoku_vision and gomoku_ai

- geometry_msgs/Pose.msg
From gomoku_vision to gomoku_control

### Hardware Architecture
- Kinova Gen3 Lite: 6DOF Robot Arm
- Intel RealSense D435: RGBD Camera
- Lego 2x2 bricks: Gomoku pieces

### Commits
Follow the format proposed in the [Semantic Commits](https://gist.github.com/joshbuchea/6f47e86d2510bce28f8e7f42ae84c716)