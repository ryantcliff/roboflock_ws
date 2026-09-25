from bring_up.meshtastic_bridge import parse_command, parse_node_id, parse_position


def position_packet(**position):
    return {'from': 1, 'decoded': {'portnum': 'POSITION_APP', 'position': position}}


def text_packet(text):
    return {'from': 1, 'decoded': {'portnum': 'TEXT_MESSAGE_APP', 'text': text}}


def test_node_id_forms():
    assert parse_node_id(1819531008) == 1819531008
    assert parse_node_id('1819531008') == 1819531008
    assert parse_node_id('!6c73c000') == 0x6c73c000


def test_position_from_integer_fields():
    lat, lon, alt, bits = parse_position(
        position_packet(latitudeI=430008000, longitudeI=-787890000, altitude=180))
    assert abs(lat - 43.0008) < 1e-9 and abs(lon + 78.789) < 1e-9
    assert alt == 180.0 and bits == 32


def test_position_from_float_fields_and_precision():
    lat, lon, _, bits = parse_position(
        position_packet(latitude=43.0, longitude=-78.8, precisionBits=13))
    assert (lat, lon, bits) == (43.0, -78.8, 13)


def test_position_rejects_no_fix_and_other_ports():
    assert parse_position(position_packet(latitudeI=0, longitudeI=0)) is None
    assert parse_position(position_packet()) is None
    assert parse_position(text_packet('home')) is None


def test_commands():
    assert parse_command(text_packet('  Home ')) == 'home'
    assert parse_command(text_packet('stop')) == 'stop'
    assert parse_command(text_packet('dance')) is None
    assert parse_command(position_packet(latitude=1.0, longitude=1.0)) is None
