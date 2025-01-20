import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import socket
import json

class IMUPublishNode(Node):
    def __init__(self):
        super().__init__('imu_receiver_node')

        # Create a UDP socket
        self.udp_ip = "0.0.0.0"
        self.udp_port = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))

        # Create a publisher for PoseArray
        self.publisher = self.create_publisher(Imu, 'imu_data', 10)

        # Timer to periodically check for data
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            data = json.loads(data.decode('utf-8'))
            
            # Extract quaternion values
            quatX = data.get('quatX', 0.0)
            quatY = data.get('quatY', 0.0)
            quatZ = data.get('quatZ', 0.0)
            quatW = data.get('quatW', 0.0)
            accX = data.get('accelX',0.0)
            accY = data.get('accelY',0.0)
            accZ = data.get('accelZ',0.0)
            gyrX = data.get('gyroX',0.0)
            gyrY = data.get('gyroY',0.0)
            gyrZ = data.get('gyroZ',0.0)
            
            # Create PoseArray message
            # Create a Pose with the extracted quaternion values
            imu = Imu()

            # Header
            imu.header.stamp = self.get_clock().now().to_msg()
            imu.header.frame_id = 'imu_sensor'            
            imu.orientation.x = quatX
            imu.orientation.y = quatY
            imu.orientation.z = quatZ
            imu.orientation.w = quatW
            imu.angular_velocity.x = gyrX
            imu.angular_velocity.y = gyrY
            imu.angular_velocity.z = gyrZ
            imu.linear_acceleration.x = accX
            imu.linear_acceleration.y = accY
            imu.linear_acceleration.z = accZ

            # Publish the PoseArray message
            self.publisher.publish(imu)

            # self.get_logger().info(f"Published PoseArray: Orientation [x:{quatX}, y:{quatY}, z:{quatZ}, w:{quatW}]")
            # self.get_logger().info(f"Published PoseArray: {data}]")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error decoding JSON: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublishNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
