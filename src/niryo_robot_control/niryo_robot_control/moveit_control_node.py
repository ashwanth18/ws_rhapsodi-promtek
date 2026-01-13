#!/usr/bin/env python3
"""
MoveIt2 Control Node for Niryo NED3 Pro Robot

This node demonstrates various MoveIt2 capabilities:
- Joint space planning
- Cartesian planning
- Pose target planning
- Trajectory execution
- Service-based control

Author: Learning ROS 2 Robotics
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import moveit.core
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import std_srvs.srv
import sensor_msgs.msg
import trajectory_msgs.msg

import numpy as np
import time
import threading
from typing import List, Optional


class NiryoMoveItControl(Node):
    """
    MoveIt2 control node for Niryo NED3 Pro robot.
    """
    
    def __init__(self):
        super().__init__('niryo_moveit_control')
        
        self.get_logger().info("Initializing Niryo MoveIt Control Node")
        
        # Initialize MoveIt commander
        self.moveit_commander = moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Get the planning group
        self.group_name = "niryo_ned3pro_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        
        # Set planning parameters
        self.move_group.set_planning_time(5.0)
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.5)
        
        # Get robot information
        self.get_logger().info(f"Planning frame: {self.move_group.get_planning_frame()}")
        self.get_logger().info(f"End effector link: {self.move_group.get_end_effector_link()}")
        self.get_logger().info(f"Available planning groups: {self.robot.get_group_names()}")
        
        # Create callback group for services
        self.callback_group = ReentrantCallbackGroup()
        
        # Create service for manual control
        self.manual_control_service = self.create_service(
            std_srvs.srv.Trigger,
            'execute_demo',
            self.manual_control_callback,
            callback_group=self.callback_group
        )
        
        # Create timer for periodic demo
        self.timer = self.create_timer(15.0, self.execute_demo_sequence)
        
        # Demo sequence state
        self.demo_running = False
        self.demo_lock = threading.Lock()
        
        self.get_logger().info("Niryo MoveIt Control Node initialized successfully!")
        
    def execute_demo_sequence(self):
        """Execute the complete demo sequence."""
        with self.demo_lock:
            if self.demo_running:
                return
            self.demo_running = True
            
        try:
            self.get_logger().info("=== Starting Demo Sequence ===")
            
            # 1. Move to home position
            self.move_to_home_position()
            
            # 2. Joint space planning
            self.execute_joint_space_motion()
            
            # 3. Cartesian planning
            self.execute_cartesian_motion()
            
            # 4. Pose target planning
            self.execute_pose_target_motion()
            
            # 5. Back to home
            self.move_to_home_position()
            
            self.get_logger().info("=== Demo Sequence Completed ===")
            
        except Exception as e:
            self.get_logger().error(f"Error in demo sequence: {str(e)}")
        finally:
            with self.demo_lock:
                self.demo_running = False
    
    def move_to_home_position(self):
        """Move robot to home position."""
        self.get_logger().info("Moving to home position...")
        
        # Define home position (adjust for your robot)
        home_joint_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        try:
            self.move_group.set_joint_value_target(home_joint_values)
            plan = self.move_group.plan()
            
            if plan[0]:
                self.get_logger().info("Planning to home position successful")
                self.move_group.execute(plan[1])
                self.get_logger().info("Moved to home position")
            else:
                self.get_logger().error("Failed to plan to home position")
                
        except Exception as e:
            self.get_logger().error(f"Error moving to home: {str(e)}")
    
    def execute_joint_space_motion(self):
        """Execute joint space motion."""
        self.get_logger().info("Executing joint space motion...")
        
        # Define target joint positions (adjust for your robot)
        target_joint_values = [0.5, 0.3, -0.2, 0.0, 0.5, 0.0]
        
        try:
            self.move_group.set_joint_value_target(target_joint_values)
            plan = self.move_group.plan()
            
            if plan[0]:
                self.get_logger().info("Joint space planning successful")
                self.move_group.execute(plan[1])
                self.get_logger().info("Joint space motion executed")
            else:
                self.get_logger().error("Failed to plan joint space motion")
                
        except Exception as e:
            self.get_logger().error(f"Error in joint space motion: {str(e)}")
    
    def execute_cartesian_motion(self):
        """Execute Cartesian motion."""
        self.get_logger().info("Executing Cartesian motion...")
        
        try:
            # Get current pose
            current_pose = self.move_group.get_current_pose()
            
            # Define waypoints for Cartesian path
            waypoints = []
            
            # Start from current pose
            target_pose = current_pose.pose
            
            # Move up
            target_pose.position.z += 0.1
            waypoints.append(target_pose)
            
            # Move forward
            target_pose.position.x += 0.1
            waypoints.append(target_pose)
            
            # Move down
            target_pose.position.z -= 0.1
            waypoints.append(target_pose)
            
            # Move back
            target_pose.position.x -= 0.1
            waypoints.append(target_pose)
            
            # Execute Cartesian path
            (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints, 0.01, 0.0  # waypoints, eef_step, jump_threshold
            )
            
            if fraction > 0.8:
                self.get_logger().info(f"Cartesian path computed successfully ({fraction*100:.2f}% achieved)")
                self.move_group.execute(plan)
                self.get_logger().info("Cartesian motion executed")
            else:
                self.get_logger().error(f"Failed to compute Cartesian path ({fraction*100:.2f}% achieved)")
                
        except Exception as e:
            self.get_logger().error(f"Error in Cartesian motion: {str(e)}")
    
    def execute_pose_target_motion(self):
        """Execute pose target motion."""
        self.get_logger().info("Executing pose target motion...")
        
        try:
            # Define target pose
            target_pose = geometry_msgs.msg.Pose()
            target_pose.position.x = 0.3
            target_pose.position.y = 0.2
            target_pose.position.z = 0.4
            target_pose.orientation.w = 1.0
            target_pose.orientation.x = 0.0
            target_pose.orientation.y = 0.0
            target_pose.orientation.z = 0.0
            
            self.move_group.set_pose_target(target_pose)
            plan = self.move_group.plan()
            
            if plan[0]:
                self.get_logger().info("Pose target planning successful")
                self.move_group.execute(plan[1])
                self.get_logger().info("Pose target motion executed")
            else:
                self.get_logger().error("Failed to plan pose target motion")
                
        except Exception as e:
            self.get_logger().error(f"Error in pose target motion: {str(e)}")
    
    def manual_control_callback(self, request, response):
        """Handle manual control service requests."""
        self.get_logger().info("Manual control triggered")
        
        # Start demo in a separate thread to avoid blocking
        demo_thread = threading.Thread(target=self.execute_demo_sequence)
        demo_thread.start()
        
        response.success = True
        response.message = "Demo sequence started"
        return response
    
    def get_current_joint_values(self):
        """Get current joint values."""
        return self.move_group.get_current_joint_values()
    
    def get_current_pose(self):
        """Get current end effector pose."""
        return self.move_group.get_current_pose()
    
    def add_collision_object(self, name, pose, size):
        """Add a collision object to the planning scene."""
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.move_group.get_planning_frame()
        box_pose.pose = pose
        
        self.scene.add_box(name, box_pose, size)
        self.get_logger().info(f"Added collision object: {name}")
    
    def remove_collision_object(self, name):
        """Remove a collision object from the planning scene."""
        self.scene.remove_world_object(name)
        self.get_logger().info(f"Removed collision object: {name}")


def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    node = NiryoMoveItControl()
    
    # Create executor with multiple threads
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        node.get_logger().info("Niryo MoveIt Control Node started")
        node.get_logger().info("Demo sequence will run every 15 seconds")
        node.get_logger().info("You can also trigger it manually via the /execute_demo service")
        
        executor.spin()
        
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        # Cleanup
        moveit_commander.roscpp_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main() 