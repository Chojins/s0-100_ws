#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time

import scservo_sdk
from scservo_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncRead,
    GroupSyncWrite,
    COMM_SUCCESS,
)

PROTOCOL_VERSION = 0
BAUDRATE = 1_000_000
DEVICE_NAME = '/dev/ttyACM0'  # Adjust this to match your port

# Define your servo IDs
SERVO_IDS = [1, 2, 3, 4, 5, 6]  # Adjust these to match your servo IDs
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # Adjust names

class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')
        
        # Initialize PortHandler instance
        self.port_handler = PortHandler(DEVICE_NAME)
        
        # Initialize PacketHandler instance
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        
        # Open port
        if self.port_handler.openPort():
            self.get_logger().info("Succeeded to open the port")
        else:
            self.get_logger().error("Failed to open the port")
            return

        # Set port baudrate
        if self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().info("Succeeded to change the baudrate")
        else:
            self.get_logger().error("Failed to change the baudrate")
            return

        # Initialize publishers and subscribers
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Timer for publishing current positions
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def read_position(self, servo_id):
        """Read the current position of a servo"""
        position, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, servo_id, 56  # 56 is the address for Present_Position
        )
        if result != COMM_SUCCESS:
            self.get_logger().error(f"Failed to read position from ID {servo_id}")
            return None
        return position

    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        
        # Read positions from all servos
        positions = []
        for servo_id in SERVO_IDS:
            pos = self.read_position(servo_id)
            if pos is not None:
                # Convert from servo units to radians (adjust conversion as needed)
                pos_rad = float(pos) * (3.14159 / 2048.0)  # Assuming 0-4095 range maps to 0-2π
                positions.append(pos_rad)
            else:
                positions.append(0.0)
        
        msg.position = positions
        msg.velocity = []  # Optional: Add velocities if needed
        msg.effort = []    # Optional: Add efforts if needed
        
        self.publisher.publish(msg)

    def joint_state_callback(self, msg):
        """Handle incoming joint commands"""
        for name, position in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                servo_id = SERVO_IDS[JOINT_NAMES.index(name)]
                # Convert position from radians to servo units
                servo_pos = int(position * (2048.0 / 3.14159))  # Convert radians to servo units
                result, error = self.packet_handler.write2ByteTxRx(
                    self.port_handler, 
                    servo_id, 
                    42,  # 42 is the address for Goal_Position
                    servo_pos
                )
                if result != COMM_SUCCESS:
                    self.get_logger().error(f"Failed to write position to ID {servo_id}")

def main(args=None):
    rclpy.init(args=args)
    servo_driver = ServoDriver()
    rclpy.spin(servo_driver)
    servo_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
