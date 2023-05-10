import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from yolov8_msgs.msg import Yolov8Inference
import math
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__('person_follower_node')

        self.yolobot_odom = None

        self.yolov8_inference_subscriber = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.yolov8_inference_callback,
            10)

        self.yolobot_odom_subscriber = self.create_subscription(
            Odometry,
            '/yolobot/odom',
            self.yolobot_odom_callback,
            10)

        self.box_bot_cmd_vel_publishers = []
        for i in range(5):
            publisher = self.create_publisher(Twist, f'/box_bot{i}/cmd_vel', 10)
            self.box_bot_cmd_vel_publishers.append(publisher)

    def yolov8_inference_callback(self, inference_msg):
        person_detected = any(
            obj.class_name == 'person' for obj in inference_msg.yolov8_inference)

        if person_detected and self.yolobot_odom is not None:
            self.send_box_bots_to_yolobot()

    def yolobot_odom_callback(self, odom_msg):
        self.yolobot_odom = odom_msg

    def send_box_bots_to_yolobot(self):
        for i, publisher in enumerate(self.box_bot_cmd_vel_publishers):
            # Get the box_bot's current odometry data
            box_bot_odom = self.get_box_bot_odom(i)
            if box_bot_odom is None:
                continue

            # Calculate the position error
            error_x = self.yolobot_odom.pose.pose.position.x - box_bot_odom.pose.pose.position.x
            error_y = self.yolobot_odom.pose.pose.position.y - box_bot_odom.pose.pose.position.y

            # Calculate the angle error
            box_bot_yaw = self.get_yaw_from_quaternion(box_bot_odom.pose.pose.orientation)
            target_yaw = math.atan2(error_y, error_x)
            error_yaw = self.normalize_angle(target_yaw - box_bot_yaw)

            # Apply proportional control
            kp_linear = 0.5  # Adjust this value to change the speed response
            kp_angular = 1.5  # Adjust this value to change the turning response

            twist = Twist()
            twist.linear.x = kp_linear * math.sqrt(error_x*2 + error_y*2)
            twist.angular.z = kp_angular * error_yaw

            # Limit the maximum linear and angular velocities
            max_linear_velocity = 0.5
            max_angular_velocity = 1.0
            twist.linear.x = min(max(twist.linear.x, -max_linear_velocity), max_linear_velocity)
            twist.angular.z = min(max(twist.angular.z, -max_angular_velocity), max_angular_velocity)

            publisher.publish(twist)
    
    def get_box_bot_odom(self, index):
        try:
            tf_buffer = self.get_clock().now()
            transform = self.tf_buffer.lookup_transform(f'box_bot{index}', 'yolobot', tf_buffer)
            return transform.transform
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warning(f"Error getting box_bot{index} odometry: {e}")
            return None

    def get_yaw_from_quaternion(self, quaternion):
        _, _, yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return yaw

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi






    
    
def main(args=None):
    rclpy.init(args=args)
    person_follower_node = PersonFollowerNode()

    rclpy.spin(person_follower_node)

    person_follower_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
