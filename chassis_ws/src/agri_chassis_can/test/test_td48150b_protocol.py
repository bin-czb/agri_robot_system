from agri_chassis_can.td48150b_protocol import TD48150BProtocol


def test_enable_frames():
    assert TD48150BProtocol.enable(1) == bytes.fromhex("23 0D 20 01 00 00 00 00")
    assert TD48150BProtocol.enable(2) == bytes.fromhex("23 0D 20 02 00 00 00 00")


def test_speed_positive_example():
    assert TD48150BProtocol.speed_command(1, 1000) == bytes.fromhex(
        "23 00 20 01 00 00 03 E8"
    )


def test_speed_negative_twos_complement():
    assert TD48150BProtocol.speed_command(1, -1000) == bytes.fromhex(
        "23 00 20 01 FF FF FC 18"
    )
    assert TD48150BProtocol.speed_command(1, -10000) == bytes.fromhex(
        "23 00 20 01 FF FF D8 F0"
    )


def test_parse_speed_feedback_signed():
    parsed = TD48150BProtocol.parse_speed_feedback(
        bytes.fromhex("60 03 21 01 FF 9C 00 C8")
    )
    assert parsed is not None
    assert parsed.channel_a == -100
    assert parsed.channel_b == 200
