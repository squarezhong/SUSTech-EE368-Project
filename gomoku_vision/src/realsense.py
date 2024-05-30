#!/usr/bin/env python3

import pyrealsense2 as rs
import numpy as np

class RealSense:
    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(self.config)

    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
        # depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        # depth_image = np.asanyarray(depth_frame.get_data())
        return color_image

    def stop(self):
        self.pipeline.stop()

if __name__ == '__main__':
    import cv2

    camera = RealSense()
    try:
        while True:
            color_image = camera.get_frames()
            cv2.imshow('RealSense', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        camera.stop()
        cv2.destroyAllWindows()
