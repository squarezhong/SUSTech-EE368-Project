import cv2
import numpy as np

# callback function, handle mouse events
def show_hsv(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        hsv_value = hsv[y, x]
        h, s, v = hsv_value
        text = f'HSV: ({h}, {s}, {v})'
        img_copy = image.copy()
        cv2.putText(img_copy, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Image', img_copy)

image = cv2.imread('gomoku_22.png')

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# show the image and set the mouse callback function
cv2.imshow('Image', image)
cv2.setMouseCallback('Image', show_hsv)

cv2.waitKey(0)
cv2.destroyAllWindows()