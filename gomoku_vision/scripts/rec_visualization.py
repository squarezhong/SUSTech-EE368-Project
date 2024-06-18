import cv2
import numpy as np

# read the image
image = cv2.imread('gomoku_22.png')
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# apply Gaussian blur
blurred = cv2.GaussianBlur(gray, (5, 5), 0)

# apply adaptive thresholding
thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                               cv2.THRESH_BINARY_INV, 11, 2)

# apply Canny edge detection
edges = cv2.Canny(thresh, 50, 150, apertureSize=3)

# find contours
contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

max_area = 0
max_rect = None

# find the largest rectangle
for contour in contours:
    # use polygonal approximation to reduce the number of points
    epsilon = 0.02 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    
    if len(approx) == 4:
        area = cv2.contourArea(approx)
        if area > max_area:
            max_area = area
            max_rect = approx

if max_rect is not None:
    # get the four corner points of the rectangle
    points = max_rect.reshape(4, 2)
    
    # mark the points on the image
    for point in points:
        cv2.circle(image, tuple(point), 5, (0, 0, 255), -1)

    # draw the rectangle
    cv2.polylines(image, [max_rect], True, (0, 0, 255), 2)

cv2.imshow('Detected Largest Rectangle', image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# save the image with the detected rectangle
cv2.imwrite('largest_rectangle.jpg', image)
