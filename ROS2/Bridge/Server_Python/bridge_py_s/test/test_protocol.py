import pytest

from bridge_py_s.protocol import (
    BODY_LENGTH_FIELD_LENGTH,
    FRAME_PREFIX_LENGTH,
    HEADER_LENGTH,
    MAX_BODY_LENGTH,
    ProtocolError,
    pack_frame,
    unpack_frame,
)


@pytest.mark.parametrize("body_length", [0, 1, 127, 128])
def test_frame_round_trip(body_length):
    header = bytes(range(HEADER_LENGTH))
    body = bytes(i % 256 for i in range(body_length))

    frame = pack_frame(header, body)
    parsed_header, parsed_body = unpack_frame(frame)

    assert parsed_header == header
    assert parsed_body == body
    assert len(frame) == FRAME_PREFIX_LENGTH + body_length
    assert int.from_bytes(
        frame[HEADER_LENGTH:HEADER_LENGTH + BODY_LENGTH_FIELD_LENGTH], byteorder="big"
    ) == body_length


def test_pack_rejects_oversized_body():
    with pytest.raises(ProtocolError):
        pack_frame(bytes(HEADER_LENGTH), bytes(MAX_BODY_LENGTH + 1))


def test_unpack_rejects_short_frame():
    with pytest.raises(ProtocolError):
        unpack_frame(bytes(FRAME_PREFIX_LENGTH - 1))


def test_unpack_rejects_declared_length_mismatch():
    frame = bytes(HEADER_LENGTH) + (3).to_bytes(4, byteorder="big") + b"ab"

    with pytest.raises(ProtocolError):
        unpack_frame(frame)


def test_unpack_rejects_oversized_declared_length():
    frame = (
        bytes(HEADER_LENGTH)
        + (MAX_BODY_LENGTH + 1).to_bytes(4, byteorder="big")
        + bytes(MAX_BODY_LENGTH + 1)
    )

    with pytest.raises(ProtocolError):
        unpack_frame(frame)


def test_unpack_rejects_text_frame():
    with pytest.raises(ProtocolError):
        unpack_frame("not-binary")
