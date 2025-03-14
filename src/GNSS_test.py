import rclpy
from rclpy.node import Node
import serial
from pyubx2 import UBXReader
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32

class GNSSIMUPublisher(Node):
    def __init__(self):
        super().__init__('gnss_imu_publisher')

        # Publishers
        self.gnss_pub = self.create_publisher(NavSatFix, 'gnss/fix', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.heading_pub = self.create_publisher(Float32, 'imu/heading', 10)

        # Serial Connection
        self.serial_port = "/dev/tty.usbmodem14101"  # Adjust as needed
        self.baud_rate = 115200
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        self.ubr = UBXReader(self.ser, protfilter=2)

        # Timer
        self.create_timer(0.1, self.read_sensor_data)  # 10 Hz

    def read_sensor_data(self):
        raw_data, parsed_data = self.ubr.read()

        if parsed_data:
            msg_type = parsed_data.identity

            if msg_type == "NAV-PVT":  # Fused GNSS Position & Velocity
                gnss_msg = NavSatFix()
                gnss_msg.latitude = parsed_data.lat / 1e7
                gnss_msg.longitude = parsed_data.lon / 1e7
                gnss_msg.altitude = parsed_data.hMSL / 1e3
                self.gnss_pub.publish(gnss_msg)
                self.get_logger().info(f"Published GNSS: {gnss_msg.latitude}, {gnss_msg.longitude}, {gnss_msg.altitude}")

            elif msg_type == "ESF-INS":  # Fused IMU Data
                imu_msg = Imu()
                imu_msg.orientation.x = parsed_data.roll / 1e5
                imu_msg.orientation.y = parsed_data.pitch / 1e5
                imu_msg.orientation.z = parsed_data.yaw / 1e5
                self.imu_pub.publish(imu_msg)

                heading_msg = Float32()
                heading_msg.data = parsed_data.yaw / 1e5
                self.heading_pub.publish(heading_msg)

                self.get_logger().info(f"Published IMU: Roll={imu_msg.orientation.x}, Pitch={imu_msg.orientation.y}, Yaw={imu_msg.orientation.z}")

def main(args=None):
    rclpy.init(args=args)
    node = GNSSIMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
