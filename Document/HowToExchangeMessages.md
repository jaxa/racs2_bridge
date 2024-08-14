# How to convert messages

## ROS2->cFS

### ROS2 side

#### Case for ROS2 std_msgs type

* The structure of sample app directory

    ```
    publisher_s2c/
    ├── package.xml
    ├── publisher_s2c
    │   ├── __init__.py
    │   ├── publisher_s2c_node.py
    │   └── RACS2Bridge_std_msgs_pb2.py
    ├── resource
    │   └── publisher_s2c
    ├── setup.cfg
    ├── setup.py
    └── test
        ├── test_copyright.py
        ├── test_flake8.py
        └── test_pep257.py
    ```

    * Definition of messages for bridge

        ```
        racs2_msg/
        ├── CMakeLists.txt
        ├── msg
        │   └── RACS2UserMsg.msg
        └── package.xml
        ```

        - The contents of `RACS2UserMsg.msg` are as follows

            ```
            #
            # User data format for ROS2 app in using RACS2 Bridge
            #
            # --- Header data ---
            uint32 cfs_message_id    # Message ID of cFS destination app
            uint32 body_data_length  # Body data size in byte
            # --- Body data (serialized data is preferred): array of byte ---
            # Note:
            #   - In Python: "byte" means bytes object
            #   - In C++   : "byte" means uint8_t type
            byte[] body_data
            ```

    * Import message definition

    * Add message definition import statement for protobuf

        ```
        from racs2_msg.msg import RACS2UserMsg
        from publisher_s2c.RACS2Bridge_std_msgs_pb2 import RACS2Bridge_std_msgs
        ```

    * Define publisher

        ```
        self.publisher_ = self.create_publisher(RACS2UserMsg, '/RACS2Bridge', 10)
        ```

    * Conversion to byte sequence at publish process execution

        ```
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
        self.get_logger().info( 'Publishing: [/RACS2Bridge] [%s]' % msg_body.string_data )
        self.i += 1
        ```


### cFS side

#### Case for ROS2 std_msgs type


* The structure of sample app directory

    ```
    sample_talker/
    ├── CMakeLists.txt
    └── fsw
        ├── mission_inc
        │   └── sample_talker_perfids.h
        ├── platform_inc
        │   └── sample_talker_msgids.h
        └── src
            ├── RACS2Brdige_std_msgs.pb-c.c
            ├── RACS2Brdige_std_msgs.pb-c.h
            ├── racs2_user_msg.h
            ├── sample_talker.c
            ├── sample_talker_events.h
            ├── sample_talker.h
            ├── sample_talker_msg.h
            └── sample_talker_version.h
    ```

    * Add protobuf import statement

        ```
        #include "RACS2Bridge_std_msgs.pb-c.h"
        ```

    * Define message type structure

        ```
        CFE_SB_MsgPtr_t    RACS2_UserMsgPkt_Ptr;
        ```

    * Set up a message pipe

        ```
        status = CFE_SB_CreatePipe(&SAMPLE_LISTENER_CommandPipe, SAMPLE_PIPE_DEPTH,"TO_LISTENER");
        ```

    * Set message subscribe

        ```
        status = CFE_SB_Subscribe(SAMPLE_LISTENER_MID, SAMPLE_LISTENER_CommandPipe);
        ```


    * Receive message
        ```
        status = CFE_SB_RcvMsg(&RACS2_UserMsgPkt_Ptr, SAMPLE_LISTENER_CommandPipe, 1000);
        ```


## cFS->ROS2

### ROS2 side

### cFS side

#### Case for ROS2 std_msgs type


* The structure of sample app directory

    ```
    apps/sample_talker/
    ├── CMakeLists.txt
    └── fsw
        ├── mission_inc
        │   └── sample_talker_perfids.h
        ├── platform_inc
        │   └── sample_talker_msgids.h
        └── src
            ├── RACS2Brdige_std_msgs.pb-c.c
            ├── RACS2Brdige_std_msgs.pb-c.h
            ├── racs2_user_msg.h
            ├── sample_talker.c
            ├── sample_talker_events.h
            ├── sample_talker.h
            ├── sample_talker_msg.h
            └── sample_talker_version.h
    ```

    * Add protobuf import statement

        ```
        #include "RACS2Bridge_std_msgs.pb-c.h"
        ```

    * Define message type structure

        ```
        CFE_SB_MsgPtr_t    RACS2_UserMsgPkt_Ptr;
        ```

    * Conversion to byte sequence at publish process execution

        ```
        // send message
        // set topic name
        strcpy(RACS2_UserMsgPkt.ros2_topic_name, "/Recv/RACS2Bridge");
        // define serialized body data
        void *buffer;
        int len=0;
        RACS2BridgeStdMsgs *message;
        message=(RACS2BridgeStdMsgs *)malloc(sizeof(RACS2BridgeStdMsgs));
        racs2_bridge_std_msgs__init(message);
        int string_length = 22;
        char* buf[32];
        sprintf(buf, "Message To ROS2 :%5d", count);
        message->string_data = (char *)malloc(sizeof(string_length));
        OS_printf("SAMPLE_TALKER: [Send][MsgID=0x%x][%s]\n", RACS2_BRIDGE_MID, buf);
        strncpy(message->string_data, buf, string_length);

        len = racs2_bridge_std_msgs__get_packed_size(message);
        buffer=malloc(len);
        racs2_bridge_std_msgs__pack(message, buffer);

        // set body data
        strncpy(RACS2_UserMsgPkt.body_data, buffer, len);
        // set body data length
        RACS2_UserMsgPkt.body_data_length = len;

        // send data
        CFE_SB_TimeStampMsg((CFE_SB_Msg_t *) &RACS2_UserMsgPkt);
        status = CFE_SB_SendMsg((CFE_SB_Msg_t *) &RACS2_UserMsgPkt);
        ```


#### Case for ROS2 std_msgs type

* The structure of sample app directory

    ```
    subscriber_c2s/
    ├── package.xml
    ├── resource
    │   └── subscriber_c2s
    ├── setup.cfg
    ├── setup.py
    ├── subscriber_c2s
    │   ├── __init__.py
    │   ├── RACS2Bridge_std_msgs_pb2.py
    │   └── subscriber_c2s_node.py
    └── test
        ├── test_copyright.py
        ├── test_flake8.py
        └── test_pep257.py
    ```

    * Definition of messages for bridge

        ```
        racs2_msg/
        ├── CMakeLists.txt
        ├── msg
        │   └── RACS2UserMsg.msg
        └── package.xml
        ```

        - The contents of `RACS2UserMsg.msg` are as follows

            ```
            #
            # User data format for ROS2 app in using RACS2 Bridge
            #
            # --- Header data ---
            uint32 cfs_message_id    # Message ID of cFS destination app
            uint32 body_data_length  # Body data size in byte
            # --- Body data (serialized data is preferred): array of byte ---
            # Note:
            #   - In Python: "byte" means bytes object
            #   - In C++   : "byte" means uint8_t type
            byte[] body_data
            ```

    * Import message definition

    * Add message definition import statement for protobuf

        ```
        from racs2_msg.msg import RACS2UserMsg
        from publisher_s2c.RACS2Bridge_std_msgs_pb2 import RACS2Bridge_std_msgs
        ```

    * Define subscriber

        ```
        self.subscription = self.create_subscription(
            RACS2UserMsg,
            '/Recv/RACS2Bridge',
            self.listener_callback,
            10)
        ```

