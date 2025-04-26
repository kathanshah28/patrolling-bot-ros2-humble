#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import json
import os
import termios
import tty
import sys
import threading  # Import for threading

WAYPOINTS_FILE = os.path.expanduser('~/waypoints.json')

class WaypointSaver(Node):
    def __init__(self):
        super().__init__('waypoint_saver')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            qos_profile
        )
        self.latest_pose = None
        print("Press 's' to save the current pose. Press 'q' to quit.")

        # Start ROS spinning in a separate thread
        threading.Thread(target=self.spin_ros, daemon=True).start()

        # Start input reading in another thread
        threading.Thread(target=self.read_input, daemon=True).start()

    def spin_ros(self):
        """Spin ROS in a separate thread."""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def pose_callback(self, msg):
        print("📬 Received pose message.")
        self.latest_pose = msg.pose.pose

    def read_input(self):
        """Handle keyboard input in a separate thread."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while True:
                key = sys.stdin.read(1)
                print(f"Key pressed: {key}")
                print(f"Latest pose: {self.latest_pose}")

                if key == 's' and self.latest_pose:
                    self.save_pose(self.latest_pose)
                    print("✅ Pose saved.")
                elif key == 'q':
                    print("👋 Exiting.")
                    rclpy.shutdown()
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def save_pose(self, pose):
        print("💾 Saving pose...")
        waypoint = {
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z,
            'qx': pose.orientation.x,
            'qy': pose.orientation.y,
            'qz': pose.orientation.z,
            'qw': pose.orientation.w
        }
        waypoints = []
        if os.path.exists(WAYPOINTS_FILE):
            with open(WAYPOINTS_FILE, 'r') as f:
                waypoints = json.load(f)
        waypoints.append(waypoint)
        with open(WAYPOINTS_FILE, 'w') as f:
            json.dump(waypoints, f, indent=2)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointSaver()
    try:
        while rclpy.ok():
            pass  # Keep the main thread alive
    except KeyboardInterrupt:
        print("\n🔴 Shutdown requested.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
