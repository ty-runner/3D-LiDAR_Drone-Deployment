#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import json
import serial

PORT = "/dev/ttyTHS1"
BAUD = 115200

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        self.ser = serial.Serial(PORT, BAUD, timeout=0)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/servo_angles',
            self.listener_callback,
            10)
        self.get_logger().info(f"Servo control node started on {PORT} @ {BAUD}")

    def send(self, cmd):
        msg = json.dumps(cmd)
        self.ser.write((msg + "\n").encode("utf-8"))
        self.ser.flush()

    def listener_callback(self, msg):
        pan_angle, tilt_angle = msg.data
        pan_angle = max(-90, min(90, pan_angle))
        tilt_angle = max(-45, min(90, tilt_angle))
        self.send({"T": 133, "X": pan_angle, "Y": tilt_angle, "SPD": 0, "ACC": 0})

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    servo_node = ServoControlNode()
    rclpy.spin(servo_node)
    servo_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

