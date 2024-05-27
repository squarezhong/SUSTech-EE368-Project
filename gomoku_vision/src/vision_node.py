#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from gomoku_vision.msg import Position
from realsense import RealSense
from gomoku_board import GomokuBoard, BoardState
from gomoku_cv import GomokuCV

class GomokuVisionNode:
    def __init__(self):
        self.camera = RealSense()
        self.board = GomokuBoard()
        self.pose_pub = rospy.Publisher('arm_pose', Pose, queue_size=10)
        self.position_pub = rospy.Publisher('piece_position', Position, queue_size=10)
        self.move_sub = rospy.Subscriber('next_move', Position, self.move_callback)
        # TODO: Define the home pose
        self.home_pose = Pose(0, 0, 0, 0, 0, 0, 1)
        self.rate = rospy.Rate(30)

    def run(self):
        while not rospy.is_shutdown():
            if (self.board.get_current_player() == BoardState.BLACK):
                depth_frame, color_frame = self.camera.get_frames()
                # Find the newly placed piece
                position = GomokuCV.find_new_piece(depth_frame)
            
                if position is not False:
                    print('New piece placed at:', position)
                    self.position_pub.publish(position)
            
                    # update the game board
                    self._play_and_check(position.x, position.y)
                
            self.rate.sleep()

    def move_callback(self, data):
        # Calculate and publish the pose for the robot arm
        x = data.x
        y = data.y
        pose = GomokuCV.calculate_pose(x, y)
        self.pose_pub.publish(pose)
        
        # Update the game board
        self._play_and_check(x, y)
        
    def _play_and_check(self, x, y):
        self.board.play(x, y)
        if self.board.check_win(x, y):
            print('Game over!')
            print('The winner is:', self.board.get_winner())
            self.board.reset()
            # publish a special pose to indicate the game is over
            self.pose_pub.publish(self.home_pose)

    def stop(self):
        self.camera.stop()

if __name__ == '__main__':
    rospy.init_node('gomoku_vision_node')
    node = GomokuVisionNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        node.stop()