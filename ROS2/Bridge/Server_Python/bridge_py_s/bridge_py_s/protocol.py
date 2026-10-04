HEADER_LENGTH = 32
BODY_LENGTH_FIELD_LENGTH = 4
MAX_BODY_LENGTH = 128
FRAME_PREFIX_LENGTH = HEADER_LENGTH + BODY_LENGTH_FIELD_LENGTH


class ProtocolError(ValueError):
    pass


def pack_frame(header, body):
    header = bytes(header)
    body = bytes(body)

    if len(header) != HEADER_LENGTH:
        raise ProtocolError(f"header must be {HEADER_LENGTH} bytes")
    if len(body) > MAX_BODY_LENGTH:
        raise ProtocolError(f"body exceeds {MAX_BODY_LENGTH} bytes")

    return header + len(body).to_bytes(BODY_LENGTH_FIELD_LENGTH, byteorder="big") + body


def unpack_frame(frame):
    if not isinstance(frame, (bytes, bytearray, memoryview)):
        raise ProtocolError("websocket frame must be binary")

    frame = bytes(frame)
    if len(frame) < FRAME_PREFIX_LENGTH:
        raise ProtocolError("websocket frame is shorter than the protocol prefix")

    body_length = int.from_bytes(
        frame[HEADER_LENGTH:FRAME_PREFIX_LENGTH], byteorder="big"
    )
    if body_length > MAX_BODY_LENGTH:
        raise ProtocolError(f"declared body length exceeds {MAX_BODY_LENGTH} bytes")

    expected_length = FRAME_PREFIX_LENGTH + body_length
    if len(frame) != expected_length:
        raise ProtocolError(
            f"frame length {len(frame)} does not match declared body length {body_length}"
        )

    return frame[:HEADER_LENGTH], frame[FRAME_PREFIX_LENGTH:]
