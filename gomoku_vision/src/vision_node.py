#!/usr/bin/env python3

import os
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Int8, UInt8MultiArray
from realsense import RealSense
from gomoku_board import GomokuBoard, BoardState
from gomoku_cv import GomokuCV

master_uri = None

class GomokuVisionNode:
    def __init__(self):
        self.camera = RealSense()
        self.board = GomokuBoard(8)
        self.cv = GomokuCV(self.board.get_length())
        self.point_pub = rospy.Publisher('/arm_point', Point, queue_size=10)
        self.victory_pub = rospy.Publisher('/victory', Int8, queue_size=10)
        self.position_pub = rospy.Publisher('/piece_position', UInt8MultiArray, queue_size=10)
        self.move_sub = rospy.Subscriber('/next_move', UInt8MultiArray, self.move_callback)
        self.rate = rospy.Rate(30)


    def run(self):
        while not rospy.is_shutdown():
            if (self.board.get_current_player() == BoardState.BLACK):
                color_image = self.camera.get_frames()
                # Find the newly placed piece
                position = self.cv.process(color_image)
            
                if position is not None:
                    x = position[0]
                    y = position[1]
                    
                    print('New piece placed at:', x, y)
                    # update the game board
                    self._play_and_check(x - 1, y - 1)
                    
                    # Publish the position of the newly placed black piece
                    position_black = UInt8MultiArray()
                    position_black.data = [x, y]
                    self.position_pub.publish(position_black)
                
            self.rate.sleep()

    def move_callback(self, receive):
        received_x = receive.data[0]
        received_y = receive.data[1]
        
        # Calculate and publish the pose for the robot arm
        point_x, point_y = self.cv.calculate_point(received_x, received_y)
        
        # Create a Point message and set x, y, z values
        point = Point()
        point.x = point_x
        point.y = point_y
        point.z = 0.1 # not used
        
        self.point_pub.publish(point)
        
        # Update the game board
        self._play_and_check(received_x - 1, received_y - 1)
        
    def _play_and_check(self, x, y):
        self.board.play(x, y)
        if self.board.is_game_over():
            print('Game over!')
            print('The winner is:', self.board.get_winner())
            self.board.reset()
            # publish 1 if someone wins
            victory = Int8()
            victory.data = 1
            self.victory_pub.publish(victory)

    def stop(self):
        self.camera.stop()

if __name__ == '__main__':
    # Set the ROS_MASTER_URI environment variable
    if master_uri is not None:
        os.environ['ROS_MASTER_URI'] = master_uri
        
    rospy.init_node('gomoku_vision_node')
    node = GomokuVisionNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        node.stop()