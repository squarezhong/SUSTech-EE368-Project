#!/usr/bin/env python3

import os
import rospy
from geometry_msgs.msg import Point

def main():
    # Set the ROS_MASTER_URI environment variable
    os.environ['ROS_MASTER_URI'] = 'http://192.168.31.67:11311'
    
    # Initialize the ROS node
    rospy.init_node('point_publisher')
    
    # Create a publisher to the 'point' topic
    pub = rospy.Publisher('/planc', Point, queue_size=10)
    
    # Print instructions
    print("Enter 'x' and 'y' coordinates as integers, separated by space. Type 'q' to quit.")
    
    while not rospy.is_shutdown():
        # Get user input
        user_input = input("Enter x y: ").strip()
        
        # Check if the user wants to quit
        if user_input.lower() == 'q':
            print("Quitting...")
            break
        
        try:
            # Split the input and convert to integers
            x, y = map(int, user_input.split())
            
            # Create a Point message and set x, y, z values
            point = Point(x=x, y=y, z=0)
            
            # Publish the point message
            pub.publish(point)
            
            print(f"Published Point: x={x}, y={y}, z=0")
        except ValueError:
            print("Invalid input. Please enter two integers separated by a space.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

