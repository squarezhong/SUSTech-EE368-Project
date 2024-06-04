cmake_minimum_required(VERSION 3.0.2)
project(gomoku_ai)

## 在这里添加消息生成所需的依赖项
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation  ## 添加消息生成依赖
)

## 声明消息文件
add_message_files(
  FILES
  Position.msg
)

## 指定要生成的消息依赖项
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## 声明catkin包
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
)

## 设置包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 安装Python脚本
catkin_install_python(PROGRAMS
  src/ai_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
