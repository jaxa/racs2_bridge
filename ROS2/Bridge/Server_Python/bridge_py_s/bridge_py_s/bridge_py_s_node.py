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


class _BridgePyS(Node):

    def __init__(self):
        # Initialize
        super().__init__('_BridgePyS')
        self.declare_parameter("wss_uri", '')
        self.declare_parameter("wss_port", 0)

        self.wss_uri = self.get_parameter("wss_uri").value
        self.wss_port = self.get_parameter("wss_port").value
        print("Parameter(wss_uri) : " + self.wss_uri)
        print("Parameter(wss_port) : " + str(self.wss_port))

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
                print(str(e))
                gWebSocket = None

    def do_publish(self, topic_name, aMessage):
        if not topic_name in self.publisher_info:
            print(f"Publisher for topic[{topic_name}] does not exit")
            return
        self.publisher_info[topic_name].publish(aMessage)

    def register_publisher(self, topic_name):
        if len(self.publisher_info) == 0:
            self.publisher_info[topic_name] = self.create_publisher(
                RACS2UserMsg, topic_name, 10)
            print(f"Create publisher for topic[{topic_name}]")
        else:
            if not topic_name in self.publisher_info:
                self.publisher_info[topic_name] = self.create_publisher(
                    RACS2UserMsg, topic_name, 10)


async def spin_once(aNode):
    rclpy.spin_once(aNode, timeout_sec=0)


async def ros_run(aNode):
    while True:
        await spin_once(aNode)
        await asyncio.sleep(0.0)


# ----------------------------------------------------------
gWebSocket = None


async def wss_send(websocket, message):
    print('[RACS2 Bridge] wss_send')
    await websocket.send(message)


async def wss_recv():
    global gWebSocket
    if (gWebSocket is None):
        print("WebSocket is error.")
        return
    websocket = gWebSocket
    while True:
        try:
            recv_message = await websocket.recv()
            print('WssRecv: ' + recv_message)
            topic_name = recv_message[0:32].rstrip('\x00')
            print("topic name = ", topic_name)
            gNode.register_publisher(topic_name)
            publish_message = RACS2UserMsg()
            publish_message.body_data_length = len(recv_message) - 32
            bytes_list = [elem.encode() for elem in recv_message[32:]]
            publish_message.body_data = bytes_list
            if gNode is not None:
                gNode.do_publish(topic_name, publish_message)
        except websockets.ConnectionClosedOK:
            print("Error: websockets")
            break


async def wss_accept(websocket, path):
    print('[RACS2 Bridge] wss_accept | path: %s' % path)
    global gWebSocket
    gWebSocket = websocket

    async for message in websocket:
        print('Recv: %s' % message)
        # for confirmation
        # await websocket.send( "server accepted.")
        await wss_recv()


async def wss_run(aNode):
    print('[RACS2 Bridge] wss_run')
    async with websockets.serve(wss_accept, aNode.wss_uri, aNode.wss_port):
        await asyncio.Future()


# ----------------------------------------------------------
def main(args=None):
    print('[RACS2 Bridge] bridge_py_s - main')
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
