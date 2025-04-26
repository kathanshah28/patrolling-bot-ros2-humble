#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Path to the JSON your saver node wrote
WAYPOINTS_FILE = '/home/kathan/waypoints.json'

def load_waypoints():
    with open(WAYPOINTS_FILE, 'r') as f:
        data = json.load(f)
    return data  # list of dicts with x,y,z,qx,qy,qz,qw

def make_pose(wp, node):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x = wp['x']
    p.pose.position.y = wp['y']
    p.pose.position.z = wp.get('z', 0.0)
    # assign quaternion directly:
    p.pose.orientation.x = wp['qx']
    p.pose.orientation.y = wp['qy']
    p.pose.orientation.z = wp['qz']
    p.pose.orientation.w = wp['qw']
    return p

def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()

    # load and convert all waypoints to PoseStamped
    raw_wps = load_waypoints()
    route = [make_pose(wp, navigator) for wp in raw_wps]

    # (Optional) set an initial pose for AMCL
    initial = PoseStamped()
    initial.header.frame_id = 'map'
    initial.header.stamp = navigator.get_clock().now().to_msg()
    initial.pose = route[0].pose
    print('Initial pose:', initial.pose)
    navigator.waitUntilNav2Active()
    print('Nav2 is active')
    navigator.setInitialPose(initial)

    # patrol loop
    while rclpy.ok():
        for goal in route:
            navigator.goToPose(goal)
            # wait for the task to complete
            # this is a blocking call, so it will wait until the robot reaches the goal
            # or fails
            while not navigator.isTaskComplete():
                rclpy.spin_once(navigator, timeout_sec=0.1)
            result = navigator.getResult()
            if result != TaskResult.SUCCEEDED:
                navigator.get_logger().warn('Failed to reach a waypoint')
        # reverse order for return trip
        route.reverse()

    navigator.get_logger().info('Shutting down patrol node')
    rclpy.shutdown()

if __name__ == '__main__':
    main()