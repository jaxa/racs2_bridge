# ------------------------------------------------------------------------------
# [RACS2 Bridge] bridge_py_s - bridge_py_s_node
# ------------------------------------------------------------------------------
import rclpy
from rclpy.node import Node
import asyncio
import websockets

from racs2_msg.msg import RACS2UserMsg


# ------------------------------------------------------------------------------
gNode = None

gLogger = rclpy.logging.get_logger("RACS2Bridge")

class _BridgePyS(Node):

    def __init__(self):
        # Initialize
        super().__init__('_BridgePyS')
        self.declare_parameter("wss_uri", '')
        self.declare_parameter("wss_port", 0)

        self.wss_uri = self.get_parameter("wss_uri").value
        self.wss_port = self.get_parameter("wss_port").value
        self.get_logger().info("Parameter(wss_uri) : " + self.wss_uri)
        self.get_logger().info("Parameter(wss_port) : " + str(self.wss_port))

        # Subscription
        self.subscription = self.create_subscription(
            RACS2UserMsg,
            '/RACS2Bridge',
            self.subscription_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher
        self.publisher_info = {}

    # ROS2 Topic
    async def subscription_callback(self, aMsg):
        # self.get_logger().info('Recv data')
        bMessage = RACS2UserMsg()
        bMessage.body_data = aMsg.body_data
        # fill message id into header
        msg_header = bytearray(32)
        msg_header[0:2] = aMsg.cfs_message_id.to_bytes(2, 'big')
        msg = msg_header+b''.join(aMsg.body_data)

        global gWebSocket
        if gWebSocket is not None:
            try:
                await wss_send(gWebSocket, msg)
            except Exception as e:
                self.get_logger().error(f"Failed to send message: {e}")
                gWebSocket = None

    def do_publish(self, topic_name, aMessage):
        if not topic_name in self.publisher_info:
            self.get_logger().error(f"Publisher for topic[{topic_name}] does not exit")
            return
        self.publisher_info[topic_name].publish(aMessage)

    def register_publisher(self, topic_name):
        if topic_name in self.publisher_info:
            return

        self.publisher_info[topic_name] = self.create_publisher(
            RACS2UserMsg, topic_name, 10)
        self.get_logger().info(f"Created publisher for topic[{topic_name}]")

async def spin_once(aNode):
    rclpy.spin_once(aNode, timeout_sec=0)


async def ros_run(aNode):
    while True:
        await spin_once(aNode)
        await asyncio.sleep(0.0)


# ----------------------------------------------------------
gWebSocket = None


async def wss_send(websocket, message):
    gLogger.info('wss_send')
    await websocket.send(message)


async def wss_recv(websocket):
    global gWebSocket
    gWebSocket = websocket

    if (gWebSocket is None):
        gLogger.error("WebSocket is error.")
        return
    try:
        async for recv_message in websocket:
            gLogger.info(f'WssRecv: {recv_message}')
            topic_name = recv_message[0:32].decode()
            gLogger.info(f"topic name = {topic_name}")

            if gNode is None:
                gLogger.warning("Node is not initialized yet, skipping message")
                continue
            gNode.register_publisher(topic_name)
            publish_message = RACS2UserMsg()
            publish_message.body_data_length = len(recv_message) - 32
            bytes_list = [bytes([elem]) for elem in recv_message[32:]]
            publish_message.body_data = bytes_list
            gNode.do_publish(topic_name, publish_message)
    except websockets.ConnectionClosedOK:
        gLogger.warning("Websocket closed properly.")
    except websockets.ConnectionClosedError as e:
        gLogger.error(f"Websocket closed with error: {e}")


async def wss_run(aNode):
    gLogger.info('wss_run')
    async with websockets.serve(wss_recv, aNode.wss_uri, aNode.wss_port):
        await asyncio.Future()


# ----------------------------------------------------------
def main(args=None):
    gLogger.info('bridge_py_s - main')
    rclpy.init(args=args)

    bridge_py_s = _BridgePyS()
    global gNode
    gNode = bridge_py_s

    bEventLoop = asyncio.get_event_loop()
    asyncio.ensure_future(wss_run(bridge_py_s))
    asyncio.ensure_future(ros_run(bridge_py_s))
    bEventLoop.run_forever()

    bridge_py_s.destroy_node()
    rclpy.shutdown()


# --------------------------------------
# Main
# --------------------------------------
if __name__ == '__main__':
    main()
