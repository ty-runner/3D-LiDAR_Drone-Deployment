#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import json
import time
import serial

PORT = "/dev/ttyTHS1"
BAUD = 115200
class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        #self.kit = ServoKit(channels=16)
        #self.kit.servo[15].actuation_range = 360
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/servo_angles',
            self.listener_callback,
            10)

    def send(self, ser, cmd):
        msg = json.dumps(cmd)
        ser.write((msg + "\n").encode("utf-8"))
        ser.flush()
        print(">>", msg)

    def listener_callback(self, msg):
        pan_angle, tilt_angle = msg.data
        pan_angle = max(-90, min(90, pan_angle))
        tilt_angle = max(-45, min(90, tilt_angle))
        #self.kit.servo[14].angle = pan_angle
        #self.kit.servo[15].angle = tilt_angle
        self.send(serial.Serial(PORT, BAUD, timeout=0), {"T": 133, "X": pan_angle, "Y": tilt_angle, "SPD": 0, "ACC": 0})
        self.get_logger().info(f"Set Pannn: {pan_angle}, Tilt: {tilt_angle}")

def main(args=None):
    rclpy.init(args=args)
    servo_node = ServoControlNode()
    rclpy.spin(servo_node)
    servo_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

