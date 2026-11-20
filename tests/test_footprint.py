from g1_navigation.configuration import directional_footprint

PADDED = [0.4, 0.45, 0.4, -0.45, -0.4, -0.45, -0.4, 0.45]


def test_leading_edge_moves_out_along_the_command():
    assert directional_footprint(PADDED, 0.1, 0.2) == [0.5, 0.45, 0.5, -0.45, -0.4, -0.45, -0.4, 0.45]
    assert directional_footprint(PADDED, 0.1, -0.1) == [0.4, 0.45, 0.4, -0.45, -0.5, -0.45, -0.5, 0.45]


def test_zero_or_turn_only_command_keeps_the_padded_footprint():
    assert directional_footprint(PADDED, 0.1, 0.0) == PADDED
    assert directional_footprint(PADDED, 0.1, 0.0005) == PADDED
    assert directional_footprint(PADDED, 0.0, 0.2) == PADDED
