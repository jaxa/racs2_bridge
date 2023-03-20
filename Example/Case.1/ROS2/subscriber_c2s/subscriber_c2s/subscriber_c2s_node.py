# ------------------------------------------------------------------------------
# [RACS2 Example] subscriber_c2s - subscriber_c2s_node
# ------------------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# --------------------------------------
from racs2_msg.msg import RACS2UserMsg
from subscriber_c2s.RACS2Bridge_std_msgs_pb2 import RACS2Bridge_std_msgs
# ------------------------------------------------------------------------------


class _SubscriberC2S(Node):

    def __init__(self):
        super().__init__('subscriber_c2s')
        self.subscription = self.create_subscription(
            RACS2UserMsg,
            '/Recv/RACS2Bridge',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('Subscribing: [/Recv/RACS2Bridge]')
        message = RACS2Bridge_std_msgs()
        message.ParseFromString(b''.join(msg.body_data))
        if message.HasField("string_data"):
            print(message.string_data)


def main(args=None):
    print('[RACS2 Example] subscriber_c2s - main')
    rclpy.init(args=args)

    subscriber_c2s = _SubscriberC2S()

    rclpy.spin(subscriber_c2s)

    subscriber_c2s.destroy_node()
    rclpy.shutdown()


# --------------------------------------
# Main
# --------------------------------------
if __name__ == '__main__':
    main()
