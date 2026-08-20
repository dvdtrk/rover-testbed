#!/usr/bin/env python3

# Creeates 2 ROS2 nodes, one per pico
# Each node subscribes to cmd_vel_left and cmd_vel_right and sends byte commands to pico
# Reads telemetry lines from Pico in a background thread and publishes to ROS 2 topics


# To test:

# Left:
# ros2 topic pub /cmd_vel_left geometry_msgs/msg/Twist "{linear: {x: 0.5}}" --once

# RIght:
# ros2 topic pub /cmd_vel_right geometry_msgs/msg/Twist "{linear: {x: 0.5}}" --once

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32
import serial
import threading

# Byte protocol commands (must match firmware)
CMD_FORWARD  = 0x01
CMD_BACKWARD = 0x02
CMD_STOP     = 0x03


class MotorBridgeNode(Node):

	def __init__(self, side: str, port: str, baud: int = 115200):
		super().__init__(f'motor_bridge_{side}')

		self.side = side  # 'left' or 'right'

		# Open serial port to Pico
		self.get_logger().info(f'Opening {port} for {side} motors...')
		self.serial = serial.Serial(port, baud, timeout=0.1)
		self.get_logger().info(f'{side} serial port opened')

		# Subscribers ------------------------------------------------------
		self.cmd_vel_sub = self.create_subscription(
			Twist,
			f'cmd_vel_{side}',
			self.cmd_vel_callback,
			10
		)

		# Publishers -------------------------------------------------------
		self.current_pub = self.create_publisher(
			Float32, f'motor_current_{side}', 10)

		self.encoder_front_pub = self.create_publisher(
			Int32, f'encoder_front_{side}', 10)

		self.encoder_rear_pub = self.create_publisher(
			Int32, f'encoder_rear_{side}', 10)

		self.fault_pub = self.create_publisher(
			Int32, f'motor_fault_{side}', 10)

		# Telemetry reader thread ------------------------------------------
		# Reads lines from Pico in background and publishes to ROS 2
		self.running = True
		self.reader_thread = threading.Thread(
			target=self.read_telemetry, daemon=True)
		self.reader_thread.start()

		self.get_logger().info(f'{side} motor bridge ready')

	def cmd_vel_callback(self, msg: Twist):
		# Convert Twist linear.x to forward/backward/stop byte command
		speed = msg.linear.x

		if speed > 0.05:
			self.serial.write(bytes([CMD_FORWARD]))
		elif speed < -0.05:
			self.serial.write(bytes([CMD_BACKWARD]))
		else:
			self.serial.write(bytes([CMD_STOP]))

	def read_telemetry(self):
		# Reads telemetry lines from Pico and publishes to ROS 2 topics
		# Format: "Current: 0.43 A | EncFront: 0 EncRear: 0"
		while self.running:
			try:
				line = self.serial.readline().decode('utf-8').strip()
				if not line:
					continue

				# Parse telemetry line
				# Expected: "Current: X.XX A | EncFront: N EncRear: N"
				if 'Current:' in line and 'EncFront:' in line:
					parts = line.replace('|', '').split()
					# parts: ['Current:', '0.43', 'A', 'EncFront:', '0', 'EncRear:', '0']
					current    = float(parts[1])
					enc_front  = int(parts[4])
					enc_rear   = int(parts[6])

					# Publish current
					current_msg = Float32()
					current_msg.data = current
					self.current_pub.publish(current_msg)

					# Publish encoder counts
					front_msg = Int32()
					front_msg.data = enc_front
					self.encoder_front_pub.publish(front_msg)

					rear_msg = Int32()
					rear_msg.data = enc_rear
					self.encoder_rear_pub.publish(rear_msg)

			except Exception as e:
				self.get_logger().warn(f'Telemetry parse error: {e}')

	def destroy_node(self):
		self.running = False
		self.serial.write(bytes([CMD_STOP]))
		self.serial.close()
		super().destroy_node()


def main(args=None):
	rclpy.init(args=args)

	# Create nodes for both sides
	# Ports determined by which USB adapter is which
	left_node  = MotorBridgeNode('left',  '/dev/ttyUSB0')
	right_node = MotorBridgeNode('right', '/dev/ttyUSB1')

	# Spin both nodes using a MultiThreadedExecutor
	executor = rclpy.executors.MultiThreadedExecutor()
	executor.add_node(left_node)
	executor.add_node(right_node)

	try:
		executor.spin()
	except KeyboardInterrupt:
		pass
	finally:
		left_node.destroy_node()
		right_node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
