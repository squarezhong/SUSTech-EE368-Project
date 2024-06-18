#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point
from gomoku_vision.msg import Position

class GomokuCV:
    def __init__(self, N=8):
        self.N = N 
        self.prev_board = np.zeros((N, N))
        
        # set the points of the board corners
        self.point11 = np.array([0.303, -0.002])
        self.point18 = np.array([0.305, 0.182])
        self.point81 = np.array([0.477, -0.008])
        self.point88 = np.array([0.482, 0.182])

    def detect_board(self, image):
        """detect the board in the image to see if it is a valid board

        Args:
            image (MatLike): the image to be processed

        Returns:
            MatLike: coordinates of the rectangle that represents the board
        """
        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply adaptive thresholding
        thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                    cv2.THRESH_BINARY_INV, 11, 2)

        # Apply Canny edge detection
        edges = cv2.Canny(thresh, 50, 150, apertureSize=3)

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        max_area = 0
        max_rect = None

        # Find the largest rectangle
        for contour in contours:
            # Approximate the contour
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Check if the contour is a rectangle
            if len(approx) == 4:
                area = cv2.contourArea(approx)
                if area > max_area:
                    max_area = area
                    max_rect = approx
        
        # If a rectangle is found, return the rectangle
        if max_rect is not None:
            return max_rect
        else:
            return None
    
    
    def perspective_transform(self, image, max_rect):
        """perform perspective transformation on the image

        Args:
            image (MatLike): the image to be transformed
            max_rect (MatLike): the rectangle that represents the board

        Returns:
            MatLike: the transformed image
        """
        if max_rect is not None:
            # Reshape the rectangle to get the four corner points
            points = max_rect.reshape(4, 2)
            
            # Sort the points to get the correct order: top-left, top-right, bottom-right, bottom-left
            points = sorted(points, key=lambda x: (x[1], x[0]))
            if points[0][0] > points[1][0]:
                points[0], points[1] = points[1], points[0]
            if points[2][0] < points[3][0]:
                points[2], points[3] = points[3], points[2]

            # Calculate the width and height of the target image
            width = int(max(np.linalg.norm(points[0] - points[1]), np.linalg.norm(points[2] - points[3])))
            height = int(max(np.linalg.norm(points[0] - points[3]), np.linalg.norm(points[1] - points[2])))

            # four corners of the target image
            dst_points = np.array([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]], dtype="float32")

            # calculate the perspective transformation matrix
            M = cv2.getPerspectiveTransform(np.array(points, dtype="float32"), dst_points)

            # apply the perspective transformation
            warped = cv2.warpPerspective(image, M, (width, height))

            return warped, M
        else:
            print("No rectangle found.")
            
            
    def get_board_matrix(image):
        """transform warped image to 8x8 matrix that represents the board

        Args:
            image (MatLike): image after perspective transformation

        Returns:
            np.ndarray: matrix that represents the board
        """
        # Convert the image to HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges for black and white pieces in HSV space
        lower_black = np.array([85, 20, 50])
        upper_black = np.array([105, 130, 70])
        lower_white = np.array([80, 0, 205])
        upper_white = np.array([110, 60, 255])

        # Threshold the image to get only black and white pieces
        mask_black = cv2.inRange(hsv_image, lower_black, upper_black)
        mask_white = cv2.inRange(hsv_image, lower_white, upper_white)

        # Prepare the 8x8 matrix
        board_matrix = np.zeros((8, 8), dtype=int)

        # Calculate the size of each cell
        cell_width = image.shape[1] // 8
        cell_height = image.shape[0] // 8

        # Detect pieces within each cell
        for row in range(8):
            for col in range(8):
                cell_x = col * cell_width
                cell_y = row * cell_height
                cell_black = mask_black[cell_y:cell_y + cell_height, cell_x:cell_x + cell_width]
                cell_white = mask_white[cell_y:cell_y + cell_height, cell_x:cell_x + cell_width]

                # Count the number of black and white pixels
                black_pixel_count = np.sum(cell_black > 0)
                white_pixel_count = np.sum(cell_white > 0)

                # Determine the piece type based on the pixel counts
                if black_pixel_count > white_pixel_count:
                    if black_pixel_count > (cell_width * cell_height * 0.1):  # Threshold to ensure a piece is detected
                        board_matrix[row, col] = 1
                elif white_pixel_count > black_pixel_count:
                    if white_pixel_count > (cell_width * cell_height * 0.1):  # Threshold to ensure a piece is detected
                        board_matrix[row, col] = 2
                        
        # Rotate the matrix 90 degrees clockwise
        rotated_matrix = np.rot90(board_matrix, k=1)
        
        return rotated_matrix

    def detect_black_piece(self, current_board_matrix):
        """Compare the difference between the current board matrix 
        and the previous board matrix to detect the newly placed black piece

        Args:
            board_matrix (MatLike): matrix that represents the current board
        """
        
        # Subtract the matrices
        difference_matrix = current_board_matrix - self.prev_board

        # Find the position of the newly placed black piece
        new_black_piece_position = np.argwhere(difference_matrix == 1)
        
        self.prev_board = current_board_matrix
        
        return (new_black_piece_position[0] + 1) if new_black_piece_position.size > 0 else None
        
    def calculate_point(self, row, col):
        """use the row and column to calculate the point in the real world
        refer to the robotic arm base frame

        Args:
            row (int): row number (start from 1)
            col (int): column number (start from 1)

        Returns:
            float: x and y coordinates in the base frame
        """
        # Calculate the side length of the large square
        side_length = np.linalg.norm(self.point18 - self.point11)
        
        # Calculate the side length of each small square
        small_square_length = side_length / 8
        
        # Calculate the rotation angle of the large square
        angle = np.arctan2(self.point18[1] - self.point11[1], self.point18[0] - self.point11[0])
        
        # Calculate the offset relative to the top-left corner (point11)
        offset_x = (col - 1) * small_square_length + small_square_length / 2
        offset_y = (row - 1) * small_square_length + small_square_length / 2
        offset_y = -offset_y
        
        # Rotate the offset back to the original coordinate system
        offset = np.array([offset_x * np.cos(angle) - offset_y * np.sin(angle),
                        offset_x * np.sin(angle) + offset_y * np.cos(angle)])
        
        # Calculate the center point of the small square
        center_point = self.point11 + offset
        
        return center_point
    
    
    def process(self, color_image):
        """integrate the functions to process the image

        Args:
            color_image (MatLike): the image to be processed

        Returns:
            NDArray: the position of the newly placed black piece
        """
        if color_image is None:
            return

        # Detect the board in the image
        max_rect = self.detect_board(color_image)
        
        # Perform perspective transformation
        if max_rect is not None:
            warped, M = self.perspective_transform(color_image, max_rect)
            
            # Get the board matrix
            board_matrix = self.get_board_matrix(warped)
            
            # Detect the newly placed black piece
            new_black_piece_position = self.detect_black_piece(board_matrix)
            
            return new_black_piece_position
        
        return False
            
    