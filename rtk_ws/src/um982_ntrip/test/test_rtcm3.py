from um982_ntrip.rtcm3 import Rtcm3Inspector, crc24q


def make_frame(message_type, extra_payload=b''):
    payload = bytes([
        (message_type >> 4) & 0xff,
        (message_type & 0x0f) << 4,
    ]) + extra_payload
    header = bytes([
        0xd3,
        (len(payload) >> 8) & 0x03,
        len(payload) & 0xff,
    ])
    body = header + payload
    return body + crc24q(body).to_bytes(3, byteorder='big')


def test_fragmented_rtcm_frame_is_recovered():
    frame = make_frame(1077, b'abc')
    inspector = Rtcm3Inspector()

    assert inspector.feed(frame[:4]) == []
    assert inspector.feed(frame[4:]) == [frame]
    assert inspector.status()['message_types'] == {'1077': 1}


def test_multiple_frames_and_noise_are_recovered():
    first = make_frame(1005)
    second = make_frame(1127)
    inspector = Rtcm3Inspector()

    assert inspector.feed(b'noise' + first + second) == [first, second]
    assert inspector.valid_frames == 2
    assert inspector.discarded_bytes == 5


def test_bad_crc_is_rejected_and_next_frame_is_recovered():
    bad = bytearray(make_frame(1087))
    bad[-1] ^= 0xff
    good = make_frame(1097)
    inspector = Rtcm3Inspector()

    assert inspector.feed(bytes(bad) + good) == [good]
    assert inspector.invalid_crc == 1
    assert inspector.message_types == {1097: 1}
