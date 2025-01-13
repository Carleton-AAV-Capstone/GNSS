# GNSS Code to send to a ROS2 node via PyGPSClient
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from PyGPSClient.gpsclient import GPSClient
from PyGPSClient.gpsparser import NMEAMessage


class RTKPublisher(Node):
    def __init__(self):
        super().__init__('rtk_publisher')

        # Declare ROS 2 parameters
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('serial_port', '/dev/ttyACM0'), # Will have to change this port a few times, for now it is configured to a USB-C port on Linux
                                    ('baud_rate', 115200), # Might have to change Baud rate in the future
                                    ('gnss_topic', '/rtk_fix'), # ROS2 node that it will be called
                                ])

        # Read parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.gnss_topic = self.get_parameter('gnss_topic').get_parameter_value().string_value

        # Create ROS 2 publisher
        self.gnss_pub = self.create_publisher(NavSatFix, self.gnss_topic, 10)

        # Initialize PyGPSClient
        self.gps_client = GPSClient(self.serial_port, self.baud_rate, self.on_nmea_message)

        # Start GPS client
        self.get_logger().info('Starting PyGPSClient...')
        self.gps_client.start()

    def on_nmea_message(self, message: NMEAMessage):
        """
        Callback function to handle NMEA messages from PyGPSClient.
        """
        if message.sentence_type == 'GGA':  # RTK data is typically in GGA sentences
            self.publish_rtk_data(message)

    def publish_rtk_data(self, message):
        """
        Convert NMEA GGA message to NavSatFix and publish to ROS 2.
        """
        navsat_msg = NavSatFix()
        navsat_msg.header.stamp = self.get_clock().now().to_msg()
        navsat_msg.latitude = message.latitude
        navsat_msg.longitude = message.longitude
        navsat_msg.altitude = message.altitude

        # Fix status: 0 (no fix), 1 (GPS fix), 2 (DGPS fix, including RTK Float/Fixed)
        navsat_msg.status.status = int(message.gps_qual)
        navsat_msg.status.service = 1  # GPS

        # Publish the message
        self.gnss_pub.publish(navsat_msg)
        self.get_logger().info(f'Published RTK Data: {navsat_msg.latitude}, {navsat_msg.longitude}, {navsat_msg.altitude}')

    def destroy_node(self):
        """
        Override the destroy_node method to stop PyGPSClient on shutdown.
        """
        self.gps_client.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    rtk_publisher = RTKPublisher()

    try:
        rclpy.spin(rtk_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        rtk_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
