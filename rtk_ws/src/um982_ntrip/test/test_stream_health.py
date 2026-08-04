from um982_ntrip.node import rtcm_stream_stalled


def test_stream_without_first_payload_is_not_stalled():
    assert not rtcm_stream_stalled(None, 100.0, 10.0)


def test_recent_payload_is_not_stalled():
    assert not rtcm_stream_stalled(95.0, 100.0, 10.0)


def test_expired_payload_is_stalled():
    assert rtcm_stream_stalled(90.0, 100.0, 10.0)
