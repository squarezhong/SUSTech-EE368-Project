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
        if success:
            rospy.loginfo("Reaching home joint angles...")
            # home_joints = [14, 332, 80, 270, 288, 286]
            home_joints = [13, 341, 90, 269, 288, 239]
            success &= self.reach_joint_angles(joint_positions=home_joints, tolerance=0.01)
            rospy.loginfo(success)
        
        # Reaching picking pose
        if success:
            rospy.loginfo("Reaching picking pose...")
            # picking_joints = [348, 322, 118, 272, 337, 258]
            picking_joints = [348, 329, 119, 269, 330, 214]
            success &= self.reach_joint_angles(joint_positions=picking_joints, tolerance=0.01)
            rospy.loginfo(success)

        # Open gripper
        if self.is_gripper_present and success:
            rospy.loginfo("Opening the gripper...")
            success &= self.reach_gripper_position(0.35)
            rospy.loginfo(success)

        # Downward 0.05
        if success: 
            rospy.loginfo("Downward 0.05 ...")
            actual_pose = self.get_cartesian_pose()
            # actual_pose.position.z -= 0.01
            actual_pose.position.z -= 0.05
            success &= self.reach_cartesian_pose(pose=actual_pose, tolerance=0.005, constraints=None)
            rospy.loginfo(success)

        # Close gripper
        if self.is_gripper_present and success:
            rospy.loginfo("Closing the gripper ...")
            success &= self.reach_gripper_position(0.2)
            rospy.loginfo(success)
            
        # Upward 0.1
        if success:
            rospy.loginfo("Upward 0.1...")
            # up_pose = data
            up_pose = self.get_cartesian_pose()
            up_pose.position.z += 0.1
            success &= self.reach_cartesian_pose(pose=up_pose, tolerance=0.005, constraints=None)
            rospy.loginfo(success)

        # Reaching placing pose
        if success:
            rospy.loginfo("Reaching Placing Pose...")
            placing_pose = self.get_cartesian_pose()
            placing_pose.position.x = data.x
            placing_pose.position.y = data.y
            # placing_pose.position.x -= 0.15
            # placing_pose.position.y += 0.15
            # placing_pose.position.z -= 0.2
            # # placing_pose.orientation.x += 0.05
            success &= self.reach_cartesian_pose(pose=placing_pose, tolerance=0.005, constraints=None)
            rospy.loginfo(success)

        # Downward 0.1
        if success: 
            rospy.loginfo("Downward 0.1 ...")
            actual_pose = self.get_cartesian_pose()
            actual_pose.position.z -= 0.09
            success &= self.reach_cartesian_pose(pose=actual_pose, tolerance=0.005, constraints=None)
            rospy.loginfo(success)

        # Open gripper
        if self.is_gripper_present and success:
            rospy.loginfo("Opening the gripper...")
            success &= self.reach_gripper_position(0.23)
            rospy.loginfo(success)

        # Upward 0.08
        if success:
            rospy.loginfo("Upward 0.08...")
            up_pose = self.get_cartesian_pose()
            up_pose.position.z += 0.08
            success &= self.reach_cartesian_pose(pose=up_pose, tolerance=0.005, constraints=None)
            rospy.loginfo(success)

        # Reaching Home
        if success:
            rospy.loginfo("Reaching home joint angles...")
            home_joints = [13, 341, 90, 269, 288, 239]
            success &= self.reach_joint_angles(joint_positions=home_joints, tolerance=0.01)
            rospy.loginfo(success)

        if not success:
            rospy.logerr("The example encountered an error.")


    # Moving methods
    def get_cartesian_pose(self):
        arm_group = self.arm_group
        pose = arm_group.get_current_pose()
        # rospy.loginfo("Actual cartesian pose is : ")
        # rospy.loginfo(pose.pose)
        return pose.pose

    def reach_joint_angles(self, joint_positions, tolerance):
        arm_group = self.arm_group
        for i in range(len(joint_positions)): 
            if joint_positions[i] > 180:
                joint_positions[i] -= 360
            joint_positions[i] = joint_positions[i] / 180.0 * pi
        # rospy.loginfo(joint_positions)

        self.arm_group.set_goal_joint_tolerance(tolerance)
        arm_group.set_joint_value_target(joint_positions)
        return arm_group.go(wait=True)
    
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