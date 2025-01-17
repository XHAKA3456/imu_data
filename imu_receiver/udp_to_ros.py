import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from geometry_msgs.msg import Quaternion
import socket
import json

class IMUReceiverNode(Node):
    def __init__(self):
        super().__init__('imu_receiver_node')

        # Create a UDP socket
        self.udp_ip = "0.0.0.0"
        self.udp_port = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))

        # Create a publisher for PoseArray
        self.publisher = self.create_publisher(PoseArray, 'imu_quaternion_posearray', 10)

        # Timer to periodically check for data
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            imu_data = json.loads(data.decode('utf-8'))
            
            # Extract quaternion values
            quatX = imu_data.get('quatX', 0.0)
            quatY = imu_data.get('quatY', 0.0)
            quatZ = imu_data.get('quatZ', 0.0)
            quatW = imu_data.get('quatW', 0.0)
            accX = imu_data.get('accelX',0.0)
            accY = imu_data.get('accelY',0.0)
            accZ = imu_data.get('accelZ',0.0)
            gyrX = imu_data.get('gyroX',0.0)
            gyrY = imu_data.get('gyroY',0.0)
            gyrZ = imu_data.get('gyroZ',0.0)


            # latitude = imu_data.get('latitude',0.0)
            # longtitude = imu_data.get('longtitude',0.0)

            # Create PoseArray message
            pose_array = PoseArray()

            # Create a Pose with the extracted quaternion values
            pose = Pose()
            pose.orientation.x = quatX
            pose.orientation.y = quatY
            pose.orientation.z = quatZ
            pose.orientation.w = quatW
            # pose.position.x = poseX
            # pose.position.y = poseY
            # pose.position.z = poseZ
            # Add Pose to PoseArray
            pose_array.poses.append(pose)

            # Publish the PoseArray message
            self.publisher.publish(pose_array)

            self.get_logger().info(f"Published PoseArray: Orientation [x:{quatX}, y:{quatY}, z:{quatZ}, w:{quatW}]")
            # self.get_logger().info(f"Published PoseArray: {imu_data}]")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error decoding JSON: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = IMUReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
