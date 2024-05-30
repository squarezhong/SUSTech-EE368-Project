#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point
from gomoku_vision.msg import Position

class GomokuCV:
    def __init__(self, N=8):
        self.N = N 
        self.prev_color_frame = None
        self.board_corners = None
        self.first_flag = 0
        
        # TODO
        self.position_11 = None
        self.position_1N = None
        self.position_N1 = None
        self.position_NN = None

    def detect_board(self, image):
        # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        yellow_lower = np.array([96, 128, 144])
        yellow_upper = np.array([192, 192, 208])
        mask = cv2.inRange(image, yellow_lower, yellow_upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            if len(approx) == 4:
                self.board_corners = approx
                return True
        return False
    
    def get_perspective_transform(self, image):
        if self.board_corners is None:
            return None

        pts1 = np.float32([self.board_corners[i][0] for i in range(4)])
        board_size = 640  # size of the transformed board image
        pts2 = np.float32([[0, 0], [board_size, 0], [board_size, board_size], [0, board_size]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        transformed_image = cv2.warpPerspective(image, matrix, (board_size, board_size))
        return transformed_image, matrix
    
    def get_grid_position(self, x, y):
        board_size = 640
        grid_size = board_size / self.N
        row = int(y / grid_size)
        col = int(x / grid_size)
        return (row, col)
    
    def detect_black_piece(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if 500 < area < 5000:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    return (cX, cY)
        return None    
    
    def process(self, color_image):
        if color_image is None:
            return

        if not self.detect_board(color_image):
            rospy.loginfo("Board not detected.")
            return

        transformed_image, matrix = self.get_perspective_transform(color_image)
        if transformed_image is None:
            rospy.loginfo("Perspective transformation failed.")
            return
        
        diff_image = cv2.absdiff(self.prev_color_frame, transformed_image)
        self.prev_color_frame = transformed_image
        piece_pos = self.detect_black_piece(diff_image)

        if piece_pos is None:
            rospy.loginfo("No new black piece detected.")
            return

        grid_pos = self.get_grid_position(piece_pos[0], piece_pos[1])
        if grid_pos is not None:
            rospy.loginfo(f"New black piece detected at: {grid_pos}")
            position = Position()
            position.x = grid_pos[0]
            position.y = grid_pos[1]
            return position
        
        return False

    def calculate_point(self, x, y):
        """Calculate the desired point for the robot arm

        Args:
            x (int): the x position of the piece
            y (int): the y position of the piece

        Returns:
            Point: the desired point for the robot arm
        """
        point_x = self.position_11.x + (self.position_99.x - self.position_11.x) / 8 * (x - 1)
        point_y = self.position_11.y + (self.position_99.y - self.position_11.y) / 8 * (y - 1)

        # create geometry_msgs/Point.msg
        point = Point()
        point.x = point_x
        point.y = point_y
        point.z = 0.1
        
        return point
        
    