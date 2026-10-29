# cFE 7 and ROS 2 C++ bridge

The bridge targets cFE 7.0.1 and ROS 2 Jazzy. The ROS server is implemented
in C++ with rclcpp and libwebsockets. The cFS client uses the same WebSocket
library. Protobuf and Python are not runtime dependencies of these bridges.
The older Python/protobuf examples remain in the repository for reference;
they are not required by the C/C++ bridge.

## Wire compatibility

One binary WebSocket message contains a 32-byte routing header followed by
0 to 128 opaque application bytes. There is no body padding. This is the
only change to the upstream wire layout. Maximum message size remains 160.

* ROS to cFS: the first two header bytes contain the destination MID in
  big-endian order; the remaining 30 bytes are zero.
* cFS to ROS: the header contains a topic name, zero-padded to 32 bytes.
  A name using all 32 bytes does not require a terminating NUL.
* Payload bytes are preserved, including embedded and trailing zero bytes.
  The WebSocket message length determines the body length.
* The ROS interface is still `racs2_msg/msg/RACS2UserMsg`. For transmission,
  `body_data_length` must equal the size of `body_data`.
* The SB envelope in `Common/include/racs2_user_msg.h` preserves the upstream
  topic, body length and body fields, with cFE 7 header types. It is not the
  WebSocket wire image. No full CCSDS packet or timestamp is transported.

Applications define the encoding and interpretation of the opaque body.
The bridge validates lengths, reassembles WebSocket messages and uses finite
buffers with LWS_PRE headroom. While disconnected, outbound messages are
discarded. A full 32-message transmit queue drops new messages. There is no
delivery guarantee.
The server accepts one client, as the upstream Python bridge did in practice.

## Build on Linux

Install cFE build tools, libwebsockets development files and ROS 2 Jazzy.
From the repository root, build the ROS packages with:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --merge-install --base-paths \
  ROS2/Interfaces/racs2_msg \
  ROS2/Bridge/Server_C/bridge_cpp_s
source install/local_setup.bash
```

The command selects only the canonical interface and C++ server directories;
the repository also contains legacy duplicate example packages.

## cFS mission configuration

Build the client as part of a cFE 7.0.1 mission. Make its source available in
the cFS application search path, for example:

```bash
ln -s /path/to/racs2_bridge/cFS/Bridge/Client_C/apps/racs2_bridge_client \
  /path/to/cFS/apps/racs2_bridge_client
```

Add the application to the intended CPU's app list in the mission's
`targets.cmake`. For a CPU named `cpu1`:

```cmake
list(APPEND cpu1_APPLIST racs2_bridge_client)
```

Add this entry before the first `!` in that CPU's `cfe_es_startup.scr`:

```text
CFE_APP, racs2_client, RACS2_CLIENT_Main, RACS2_CLIENT, 55, 32768, 0x0, 0;
```

Install a client configuration file at `cf/racs2_bridge_config.txt` relative
to the cFE core's working directory:

```ini
wss_uri=127.0.0.1
wss_port=8765
```

Set its endpoint to match the ROS server. The `RACS2_CONFIG` environment
variable can select an alternative configuration file path. Build and
install the mission using its normal cFE build procedure.

### Optional raw-binary echo application

To build the echo example, also make `cFS/Examples/racs2_echo` available as
`apps/racs2_echo` and add `racs2_echo` to the same CPU's app list:

```cmake
list(APPEND cpu1_APPLIST racs2_echo)
```

Add its startup entry before the first `!`:

```text
CFE_APP, racs2_echo, RACS2_ECHO_Main, RACS2_ECHO, 56, 16384, 0x0, 0;
```

The priority and stack size in these startup entries are examples; select
values appropriate to your mission.

The example receives the RACS2 user envelope at MID 0x1894 and returns the
exact binary body on topic `/Recv/RACS2Bridge`. Include the example when
checking end-to-end communication in a development mission.

## Operation

Run `ros2 run bridge_cpp_s bridge_cpp_s_node --ros-args --params-file
/path/to/params.yaml`. Parameters retain the names `wss_uri` and `wss_port`;
these configure a plain `ws://` listener, not TLS. Default is 127.0.0.1:8765.
The cFS client reads `cf/racs2_bridge_config.txt` relative to the core's working
directory, or the file selected by the optional `RACS2_CONFIG` environment
variable. Keys remain `wss_uri` and `wss_port`.

Start the ROS server as a ROS node and let cFE start its client through the
mission startup script.
The CMake application target remains `racs2_bridge_client`, but its installed
module is `racs2_client.so`, with entry point `RACS2_CLIENT_Main`, to fit the
default filename and startup-symbol limits.

## Validation

During development, the cFE 7.0.1 / Jazzy / libwebsockets 4.3.3 Linux-container
build and end-to-end echo test passed for 0, 1, 4, 20 and 128 body bytes,
including embedded zeros. Reconnection passed after independently restarting
each side. Validation used native-only OSAL permissive scheduling; target
scheduling is unchanged.
Deployment-specific integration and timing require separate validation.
