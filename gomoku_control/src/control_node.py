#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from math import pi
from geometry_msgs.msg import Point

class GomokuControlNode:
    # Init
    def __init__(self):
        self.pose_sub = rospy.Subscriber('arm_point', Point, self.point_callback)

        # MoveIt Interfaces
        moveit_commander.roscpp_initialize(sys.argv)

        try:
            # Get parameters
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            # Arm and gripper objects
            arm_group_name = "arm"
            # gripper_group_name = "gripper"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            # self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())
            

            rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())

        except Exception as e:
            print (e)
            self.is_init_success = False
        else:
            self.is_init_success = True

    def run(self):
        rospy.spin()
    
    # Callback
    def point_callback(self, data):
        rospy.loginfo("Received Point: %s", data)
        success = self.is_init_success

        # Reaching Home
        # if success:
        #     rospy.loginfo("Reaching Named Target Home...")
        #     success &= self.reach_named_position("home")
        #     print (success)
        if success:
            rospy.loginfo("Reaching home joint angles...")
            home_joints = [348, 321, 118, 271, 337, 258]
            success &= self.reach_joint_angles(joint_positions=home_joints, tolerance=0.01)
            rospy.loginfo(success)
        
        # Reaching picking pose
        # if success: 
        #     rospy.loginfo("Reaching picking pose...")
        #     # picking_pose = Pose()
        #     # picking_pose.position.x = 0.376
        #     # picking_pose.position.y = -0.004
        #     # picking_pose.position.z = 0.136
        #     # quaternion = quaternion_from_euler(90, 0, 150)
        #     # picking_pose.orientation.x = quaternion[0]
        #     # picking_pose.orientation.y = quaternion[1]
        #     # picking_pose.orientation.z = quaternion[2]
        #     # picking_pose.orientation.w = quaternion[3]
        #     picking_pose = self.get_cartesian_pose()
        #     # picking_pose.position.x += 0.08
        #     picking_pose.position.y -= 0.2
        #     picking_pose.position.z -= 0.25
        #     success &= self.reach_cartesian_pose(pose=picking_pose, tolerance=0.01, constraints=None)
        #     print (success)

        # # Open gripper
        # if self.is_gripper_present and success:
        #     rospy.loginfo("Opening the gripper...")
        #     success &= self.reach_gripper_position(0.5)
        #     print (success)

        # # Downward 0.1
        # if success: 
        #     rospy.loginfo("Downward 0.1 ...")
        #     actual_pose = self.get_cartesian_pose()
        #     actual_pose.position.z -= 0.1
        #     success &= self.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
        #     print (success)

        # # Close gripper
        # if self.is_gripper_present and success:
        #     rospy.loginfo("Closing the gripper ...")
        #     success &= self.reach_gripper_position(0.81)
        #     print (success)
            
        # # Upward 0.1
        # if success:
        #     rospy.loginfo("Upward 0.1...")
        #     # up_pose = data
        #     up_pose = self.get_cartesian_pose()
        #     up_pose.position.z += 0.1
        #     success &= self.reach_cartesian_pose(pose=up_pose, tolerance=0.01, constraints=None)
        #     print (success)

        # # Reaching placing pose
        # if success:
        #     rospy.loginfo("Reaching Placing Pose...")
        #     placing_pose = self.get_cartesian_pose()
        #     placing_pose.x = data.x
        #     placing_pose.y = data.y
        #     # placing_pose.position.x -= 0.15
        #     # placing_pose.position.y += 0.15
        #     # placing_pose.position.z -= 0.2
        #     # # placing_pose.orientation.x += 0.05
        #     success &= self.reach_cartesian_pose(pose=placing_pose, tolerance=0.01, constraints=None)
        #     print (success)

        # # Downward 0.1
        # if success: 
        #     rospy.loginfo("Downward 0.1 ...")
        #     actual_pose = self.get_cartesian_pose()
        #     actual_pose.position.z -= 0.1
        #     success &= self.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
        #     print (success)

        # # Open gripper
        # if self.is_gripper_present and success:
        #     rospy.loginfo("Opening the gripper...")
        #     success &= self.reach_gripper_position(0.77)
        #     print (success)

        # # Upward 0.1
        # if success:
        #     rospy.loginfo("Upward 0.1...")
        #     # up_pose = data
        #     up_pose = self.get_cartesian_pose()
        #     up_pose.position.z += 0.1
        #     success &= self.reach_cartesian_pose(pose=up_pose, tolerance=0.01, constraints=None)
        #     print (success)

        # # Reaching Home
        # if success:
        #     rospy.loginfo("Reaching home joint angles...")
        #     home_joints = [0, 0, pi/2, pi/4, 0, pi/2]
        #     success &= self.reach_joint_angles(joint_positions=home_joints, tolerance=0.01)
        #     print(success)

        if not success:
            rospy.logerr("The example encountered an error.")


    # Moving methods
    def get_cartesian_pose(self):
        arm_group = self.arm_group

        pose = arm_group.get_current_pose()
        rospy.loginfo("Actual cartesian pose is : ")
        rospy.loginfo(pose.pose)

        return pose.pose

    def reach_joint_angles(self, joint_positions, tolerance):
        arm_group = self.arm_group
        for p in joint_positions: 
            if p > 180:
                p -= 360
            p = p / 180.0 * pi
        
        rospy.loginfo(joint_positions)
        success = True
        # joint_positions = arm_group.get_current_joint_values()
        # rospy.loginfo("Printing current joint positions before movement:")
        # for p in joint_positions: 
        #     rospy.loginfo(p)

        self.arm_group.set_goal_joint_tolerance(tolerance)
        arm_group.set_joint_value_target(joint_positions)
        success &= arm_group.go(wait=True)

        # new_joint_positions = arm_group.get_current_joint_values()
        # rospy.loginfo("Printing current joint positions after movement :")
        # for p in new_joint_positions: 
        #     rospy.loginfo(p)
        return success
    
    def reach_cartesian_pose(self, pose, tolerance, constraints):
        arm_group = self.arm_group
        arm_group.set_goal_position_tolerance(tolerance)
        if constraints is not None:
            arm_group.set_path_constraints(constraints)

        arm_group.set_pose_target(pose)

        rospy.loginfo("Planning and going to the Cartesian Pose")
        return arm_group.go(wait=True)
    
    def reach_gripper_position(self, relative_position):
        gripper_group = self.gripper_group
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        try:
            val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
            return val
        except:
            return False 


if __name__ == '__main__':
    rospy.init_node('gomoku_control_node')
    node = GomokuControlNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
