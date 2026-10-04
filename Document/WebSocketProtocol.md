# WebSocket packet format

RACS2 uses the same binary WebSocket packet format in both directions between the ROS2 bridge and the cFS bridge.

| Offset | Size | Field |
| --- | ---: | --- |
| 0 | 32 bytes | Header |
| 32 | 4 bytes | `body_data_length`, unsigned big-endian |
| 36 | `body_data_length` bytes | Body data |

The body may contain from 0 to 128 bytes. The total WebSocket frame size is therefore exactly `36 + body_data_length` bytes.

## Header

The 32-byte header depends on the direction:

- ROS2 to cFS: bytes 0-1 contain the cFS destination message ID in big-endian order. The remaining header bytes are reserved and currently zero-filled.
- cFS to ROS2: the header contains the ROS2 topic name, zero-padded to 32 bytes.

## Length validation

Receivers must reject a frame when any of these conditions is true:

- the frame is shorter than 36 bytes;
- `body_data_length` is greater than 128;
- the actual frame size is not exactly `36 + body_data_length` bytes;
- the WebSocket message is not binary.

The `body_data_length` field used inside ROS2/cFS application messages is an application-side representation. On the WebSocket wire, the length is always encoded as the 4-byte big-endian field described above.
