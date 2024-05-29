#!usr/bin/env python3

import cv2
import numpy as np
from geometry_msgs.msg import Pose

class GomokuCV:
    def __init__(self):
        self.prev_depth_frame = None    
        self.prev_color_frame = None
        
        # TODO
        self.position_11 = None
        self.position_19 = None
        self.position_91 = None
        self.position_99 = None
        
    def find_new_piece(self, depth_frame):
        # TODO: Find the newly placed piece
        mask = cv2.inRange(depth_frame, 0, 1000)
        mask = cv2.bitwise_and(mask, self.prev_depth_frame)
        mask = cv2.bitwise_not(mask)
        self.prev_depth_frame = depth_frame
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            return False
        # Find the center of the piece
        contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(contour)
        center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
        
        return self.center2position(center)
    
    def center2position(self, center):
        """Calculate the position of the piece on the board

        Args:
            center (_type_): the center of the piece

        Returns:
            int, int: The position of the piece on the board 
        """
        # TODO
        x = center[0] // 50
        y = center[1] // 50
        return x, y
    
    def position2center(self, x, y):
        """Calculate the center of the piece

        Args:
            x (int): the x position of the piece
            y (int): the y position of the piece

        Returns:
            _type_: the center of the piece
        """
        # 
        center = (x * 50 + 25, y * 50 + 25)
        return center
    
    def calculate_pose(self, x, y):
        """Calculate the pose for the robot arm

        Args:
            x (int): the x position of the piece
            y (int): the y position of the piece

        Returns:
            Pose: the pose for the robot arm
        """
        pose_x = self.position_11.x + (self.position_99.x - self.position_11.x) / 8 * (x - 1)
        pose_y = self.position_11.y + (self.position_99.y - self.position_11.y) / 8 * (y - 1)
        # create geometry_msgs/Pose.msg
        pose = Pose()
        pose.position.x = pose_x
        pose.position.y = pose_y
        pose.position.z = 0.1
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        
        return pose
        
    