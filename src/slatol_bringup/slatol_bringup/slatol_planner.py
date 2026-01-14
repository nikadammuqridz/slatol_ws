#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import time

class SlatolPlanner(Node):
    def __init__(self):
        super().__init__('slatol_planner')
        
        # Publisher to the robot's controller
        # We send 'Trajectory' messages to tell the joints where to go
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/slatol_position_controller/joint_trajectory', 
            10
        )
        
        # State Machine Variables
        self.state = "COMPRESSION" # Initial state
        self.timer = self.create_timer(0.02, self.control_loop) # Run at 50Hz
        self.start_time = time.time()
        
        self.get_logger().info("SLATOL Hybrid Planner Initialized. Starting Jump Sequence...")

    def send_command(self, hip_pos, knee_pos, duration):
        """Helper function to send commands to the motors"""
        msg = JointTrajectory()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['hip_hfe_joint', 'knee_kfe_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [float(hip_pos), float(knee_pos)]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(duration * 1e9) # Duration in ns
        
        msg.points.append(point)
        self.publisher_.publish(msg)

    def control_loop(self):
        # Calculate time since start of this jump cycle
        elapsed = time.time() - self.start_time
        
        # --- HYBRID FINITE STATE MACHINE (FSM) ---
        
        # PHASE 1: COMPRESSION (Crouch Down)
        # 0.0s to 1.0s
        if self.state == "COMPRESSION" and elapsed < 1.0:
            # Hip straight (0.0), Knee bent (-1.5 rad)
            self.send_command(0.0, -1.5, 0.5)
            
        # PHASE 2: THRUST (Explosive Jump)
        # 1.0s to 1.2s (Fast move!)
        elif self.state == "COMPRESSION" and elapsed >= 1.0:
            self.state = "THRUST"
            self.get_logger().info("State: THRUST (Jumping!)")
            # Hip straight (0.0), Knee fully extended (0.0)
            self.send_command(0.0, 0.0, 0.1) 

        # PHASE 3: FLIGHT (Retract legs in air)
        # 1.2s to 1.5s
        elif self.state == "THRUST" and elapsed > 1.3:
            self.state = "FLIGHT"
            self.get_logger().info("State: FLIGHT (Retracting)")
            # Pull knees up slightly to avoid stubbing toe
            self.send_command(0.0, -0.5, 0.2)

        # PHASE 4: RESET (Prepare for next jump)
        # After 3.0s, reset everything
        elif elapsed > 3.0:
            self.state = "COMPRESSION"
            self.start_time = time.time() # Reset clock
            self.get_logger().info("State: RESET (Preparing next jump)")

def main(args=None):
    rclpy.init(args=args)
    node = SlatolPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()