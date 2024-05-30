#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Int8
from gomoku_vision.msg import Position
from realsense import RealSense
from gomoku_board import GomokuBoard, BoardState
from gomoku_cv import GomokuCV

class GomokuVisionNode:
    def __init__(self):
        self.camera = RealSense()
        self.board = GomokuBoard(8)
        self.cv = GomokuCV(self.board.get_length())
        self.point_pub = rospy.Publisher('/arm_point', Point, queue_size=10)
        self.victory_pub = rospy.Publisher('/victory', Int8, queue_size=10)
        self.position_pub = rospy.Publisher('/piece_position', Position, queue_size=10)
        self.move_sub = rospy.Subscriber('/next_move', Position, self.move_callback)
        self.rate = rospy.Rate(30)

    def run(self):
        while not rospy.is_shutdown():
            if (self.board.get_current_player() == BoardState.BLACK):
                color_image = self.camera.get_frames()
                # Find the newly placed piece
                position = self.cv.process(color_image)
            
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
        point = GomokuCV.calculate_point(x, y)
        self.point_pub.publish(point)
        
        # Update the game board
        self._play_and_check(x, y)
        
    def _play_and_check(self, x, y):
        self.board.play(x, y)
        if self.board.check_win(x, y):
            print('Game over!')
            print('The winner is:', self.board.get_winner())
            self.board.reset()
            # publish 1 if someone wins
            self.victory_pub.publish(1)

    def stop(self):
        self.camera.stop()

if __name__ == '__main__':
    rospy.init_node('gomoku_vision_node')
    node = GomokuVisionNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        node.stop()