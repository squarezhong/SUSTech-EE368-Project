#!/usr/bin/env python3

import os
import rospy
import numpy as np
from std_msgs.msg import Int8, UInt8MultiArray
from geometry_msgs.msg import Point
from gomoku_board import GomokuBoard, BoardState


# set the points of the board corners
point11 = np.array([1, 1])
point18 = np.array([2, 2])
point81 = np.array([0, 1])
point88 = np.array([1, 3])

point_z = 0.1

master_uri = None
# master_uri = 'http://192.168.31.67:11311'

def calculate_point(row, col):
    # Calculate the side length of the large square
    side_length = np.linalg.norm(point18 - point11)
    
    # Calculate the side length of each small square
    small_square_length = side_length / 8
    
    # Calculate the rotation angle of the large square
    angle = np.arctan2(point18[1] - point11[1], point18[0] - point11[0])
    
    # Calculate the offset relative to the top-left corner (point11)
    offset_x = (col - 1) * small_square_length + small_square_length / 2
    offset_y = (row - 1) * small_square_length + small_square_length / 2
    
    # Rotate the offset back to the original coordinate system
    offset = np.array([offset_x * np.cos(angle) - offset_y * np.sin(angle),
                       offset_x * np.sin(angle) + offset_y * np.cos(angle)])
    
    # Calculate the center point of the small square
    center_point = point11 + offset
    
    return center_point

class GomokuVisionNode:
    def __init__(self):
        self.board = GomokuBoard(8)
        self.point_pub = rospy.Publisher('/arm_point', Point, queue_size=10)
        self.victory_pub = rospy.Publisher('/victory', Int8, queue_size=10)
        self.position_pub = rospy.Publisher('/piece_position', UInt8MultiArray, queue_size=10)
        self.move_sub = rospy.Subscriber('/next_move', UInt8MultiArray, self.move_callback)
        self.rate = rospy.Rate(30)    
            
    def run(self):
        """print("Enter 'x' and 'y' coordinates as integers, separated by space. Type 'q' to quit.")
        """
        while not rospy.is_shutdown():
            if (self.board.get_current_player() == BoardState.BLACK):
                # Get user input
                user_input = input("Enter x y: ").strip()
        
                # Check if the user wants to quit
                if user_input.lower() == 'q':
                    print("Quitting...")
                    break
                
                try:
                    # Split the input and convert to integers
                    x, y = map(int, user_input.split())
                    
                    position = UInt8MultiArray()
                    position.data = [x, y]
                    self.position_pub.publish(position)
                    
                    print(f"Published Position: x={x}, y={y}")
                    
                    # update the game board
                    self._play_and_check(x - 1, y - 1)
                except ValueError:
                    print("Invalid input. Please enter two integers separated by a space.")
                    continue
                
            self.rate.sleep()

    def move_callback(self, data):
        # Calculate and publish the pose for the robot arm
        point_x, point_y = calculate_point(data.x, data.y)
            
        # Create a Point message and set x, y, z values
        point = Point(point_x, point_y, point_z)
        
        self.point_pub.publish(point)
        
        # Update the game board
        self._play_and_check(data.x - 1, data.y - 1)
        
    def _play_and_check(self, x, y):
        self.board.play(x, y)
        if self.board.check_win(x, y):
            print('Game over!')
            print('The winner is:', self.board.get_winner())
            self.board.reset()
            # publish 1 if someone wins
            self.victory_pub.publish(1)
            

if __name__ == '__main__':
    # Set the ROS_MASTER_URI environment variable
    if master_uri is not None:
        os.environ['ROS_MASTER_URI'] = master_uri

    rospy.init_node('gomoku_vision_node')
    node = GomokuVisionNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass