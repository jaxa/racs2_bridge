# ------------------------------------------------------------------------------
# [RACS2 Example] publisher_s2c - publisher_s2c_node
# ------------------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# --------------------------------------
from racs2_msg.msg import RACS2UserMsg
from publisher_s2c.RACS2Bridge_std_msgs_pb2 import RACS2Bridge_std_msgs
# ------------------------------------------------------------------------------


class _PublisherS2C(Node):

    def __init__(self):
        super().__init__('publisher_s2c')
        self.publisher_ = self.create_publisher(
            RACS2UserMsg, '/RACS2Bridge', 10)
        timer_period = (0.5)  # Second
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # define message
        msg = RACS2UserMsg()
        # define message body
        msg_body = RACS2Bridge_std_msgs()
        msg_body.string_data = 'Message S2C : %d' % self.i
        # serialize message data and fill message body
        serialized_msg_body = msg_body.SerializeToString()
        for i in range(len(serialized_msg_body)):
            msg.body_data.append(serialized_msg_body[i:i+1])
        # fill message header
        msg.cfs_message_id = 0x1894
        msg.body_data_length = len(msg.body_data)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing(S2C): %s' % msg_body.string_data)
        self.i += 1


def main(args=None):
    print('[RACS2 Example] publisher_s2c - main')
    rclpy.init(args=args)

    publisher_s2c = _PublisherS2C()

    rclpy.spin(publisher_s2c)

    publisher_s2c.destroy_node()
    rclpy.shutdown()


# --------------------------------------
# Main
# --------------------------------------
if __name__ == '__main__':
    main()
