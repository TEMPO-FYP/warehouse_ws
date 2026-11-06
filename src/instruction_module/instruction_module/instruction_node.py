"""ROS 2 Node for TEMPO Mission Instruction dispatch."""

import sys
import time

import rclpy
from rclpy.node import Node
import rclpy.utilities
from std_msgs.msg import String


def main():
    """Read mission from terminal arguments, publish it, and exit."""
    non_ros = rclpy.utilities.remove_ros_args(sys.argv)
    if len(non_ros) < 2:
        print('Error: No mission instruction provided.')
        print('Usage: ros2 run instruction_module instruction_node "<mission_instruction>"')
        sys.exit(1)

    instruction = ' '.join(non_ros[1:]).strip()

    rclpy.init()
    node = Node('instruction_node')
    pub = node.create_publisher(String, '/mission/instruction', 10)

    # Allow DDS discovery to match existing pipeline subscribers
    time.sleep(0.5)

    msg = String()
    msg.data = instruction
    pub.publish(msg)
    node.get_logger().info(f"Published mission to '/mission/instruction': '{instruction}'")

    # Give the DDS queue a moment to transmit before tearing down
    time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
