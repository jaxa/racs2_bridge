# ------------------------------------------------------------------------------
# [RACS2 Bridge] bridge_py_c - bridge_py_c_node
# ------------------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import websockets

from racs2_msg.msg import RACS2UserMsg

# ------------------------------------------------------------------------------
class _BridgePyC(Node):
    def __init__(self):
        # Initialize
        super().__init__('_BridgePyC')
        self.declare_parameter("wss_uri", '')
        self.declare_parameter("wss_port", 0)

        self.wss_uri = self.get_parameter("wss_uri").value
        self.wss_port = self.get_parameter("wss_port").value
        print( "Parameter(wss_uri) : " + self.wss_uri )
        print( "Parameter(wss_port) : " + str( self.wss_port ) )

        # Subscription
        self.subscription = self.create_subscription(
            String,
            '/Send/TopicC2S',
            self.subscription_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher
        self.publisher_ = self.create_publisher(RACS2UserMsg, '/Recv/TopicS2C', 10)

    # ROS2 Topic
    async def subscription_callback(self, aMsg):
        global gWebSocket
        if gWebSocket is not None:
            try:
                await wss_send(gWebSocket, aMsg.data)
            except Exception as e:
                print(e)
                gWebSocket = None

    def do_publish(self, aMessage):
        self.publisher_.publish(aMessage)


async def spin_once(aNode):
    rclpy.spin_once(aNode, timeout_sec=0)


async def ros_run(aNode):
    while True:
        await spin_once(aNode)
        await asyncio.sleep(0.0)


# ----------------------------------------------------------
gWebSocket = None


async def wss_send(websocket, message):
    # print('[RACS2 Bridge] wss_send')
    print('WssSend: ' + message)
    await websocket.send(message)


async def wss_offer(aNode):
    print('[RACS2 Bridge] wss_offer')

    uri = "ws://" + aNode.wss_uri + ":" + str( aNode.wss_port )
    async with websockets.connect(uri) as websocket:

        global gWebSocket
        gWebSocket = websocket

        print("offer to connect.")
        await websocket.send("offer to connect.")

        while True:
            try:
                recv_message = await websocket.recv()
                publish_message = RACS2UserMsg()
                publish_message.body_data = [bytes(elem) for elem in recv_message]
                aNode.do_publish(publish_message)
            except websockets.ConnectionClosedOK:
                print("Error: websockets.ConnectionClosedOK")
                break


async def wss_run(aNode):
    print('[RACS2 Bridge] wss_run')
    await wss_offer(aNode)


# ----------------------------------------------------------
def main(args=None):
    print('[RACS2 Bridge] bridge_py_c - main')
    rclpy.init(args=args)

    bridge_py_c = _BridgePyC()

    bEventLoop = asyncio.get_event_loop()
    asyncio.ensure_future(wss_run(bridge_py_c))
    asyncio.ensure_future(ros_run(bridge_py_c))
    bEventLoop.run_forever()

    bridge_py_c.destroy_node()
    rclpy.shutdown()


# --------------------------------------
# Main
# --------------------------------------
if __name__ == '__main__':
    main()
