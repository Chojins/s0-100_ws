#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time

from robot_arm_control.scservo_sdk import *

PROTOCOL_VERSION = 0
BAUDRATE = 1_000_000  # STS3215 uses 1Mbps
DEVICE_NAME = '/dev/ttyACM0'

# Define your servo IDs
SERVO_IDS = [1, 2, 3, 4, 5, 6]  # SO-100 uses servos 1-6
JOINT_NAMES = ['Rotation', 'Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll', 'Jaw']

# Control Table Addresses for STS3215
ADDR_TORQUE_ENABLE = 40      # 0x28
ADDR_OPERATING_MODE = 11     # 0x0B
ADDR_GOAL_POSITION = 42      # 0x2A
ADDR_PRESENT_POSITION = 56   # 0x38
ADDR_PRESENT_VOLTAGE = 62    # 0x3E

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

        # After port setup and before torque enable
        for servo_id in SERVO_IDS:
            # Read voltage to verify communication
            voltage, result, error = self.packet_handler.read1ByteTxRx(
                self.port_handler,
                servo_id,
                ADDR_PRESENT_VOLTAGE
            )
            self.get_logger().info(f"Servo {servo_id} voltage reading: {voltage/10.0}V")

            # First disable torque
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                servo_id,
                ADDR_TORQUE_ENABLE,
                0    # Disable
            )
            
            # Set to Position Control Mode
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                servo_id,
                ADDR_OPERATING_MODE,
                0    # Position Control Mode
            )
            self.get_logger().info(f"Set servo {servo_id} to position mode: result={result}, error={error}")

            # Enable torque
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                servo_id,
                ADDR_TORQUE_ENABLE,
                1    # Enable
            )
            self.get_logger().info(f"Torque enable result for servo {servo_id}: {result}, error: {error}")

        # Change publisher to publish directly to joint_states
        self.publisher = self.create_publisher(
            JointState, 
            'joint_states',
            10
        )

        # Timer for publishing current positions
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Add subscriber for joint commands
        self.command_subscription = self.create_subscription(
            JointState,
            'joint_commands',
            self.command_callback,
            10
        )

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
                # Convert from servo units to radians
                pos_rad = float(pos) * (3.14159 / 2048.0) - 3.14159
                positions.append(pos_rad)
            else:
                positions.append(0.0)
        
        msg.position = positions
        msg.velocity = []
        msg.effort = []
        
        self.publisher.publish(msg)

    def command_callback(self, msg):
        """Handle incoming joint commands"""
        try:
            self.get_logger().info("Received command message")
            for i, (position, name) in enumerate(zip(msg.position, msg.name)):
                if i < len(SERVO_IDS):
                    servo_id = SERVO_IDS[i]
                    
                    # Convert from radians to servo position (corrected formula)
                    # 2048 is center, 2048 units = 180 degrees = π radians
                    pos = int(2048 + (position * 2048 / 3.14159))
                    pos = max(0, min(4095, pos))  # Limit range
                    
                    self.get_logger().info(f"Converting {position} radians to servo position {pos}")
                    
                    # Write goal position
                    result, error = self.packet_handler.write2ByteTxRx(
                        self.port_handler, 
                        servo_id, 
                        ADDR_GOAL_POSITION,
                        pos
                    )
                    self.get_logger().info(f"Write result: {result}, error: {error}")
                    
                    time.sleep(0.1)
                    
                    # Read position to verify
                    new_pos, result, error = self.packet_handler.read2ByteTxRx(
                        self.port_handler,
                        servo_id,
                        ADDR_PRESENT_POSITION
                    )
                    self.get_logger().info(f"Position after move: {new_pos}")
                    
        except Exception as e:
            self.get_logger().error(f"Error in command callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    servo_driver = ServoDriver()
    rclpy.spin(servo_driver)
    servo_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
