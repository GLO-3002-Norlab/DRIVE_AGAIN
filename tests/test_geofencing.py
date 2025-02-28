import pytest

from DRIVE_AGAIN.geofencing import Geofence


@pytest.fixture
def geofence():
    """Set up a geofence for testing"""
    polygon_coords = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]  # A square
    return Geofence(polygon_coords)


def test_point_inside(geofence: Geofence):
    """Test if a point inside the geofence is correctly identified."""
    assert geofence.is_point_inside((2, 2))
    assert geofence.is_point_inside((3, 3))


def test_point_outside(geofence: Geofence):
    """Test if a point outside the geofence is correctly identified."""
    assert not geofence.is_point_inside((5, 5))
    assert not geofence.is_point_inside((-1, -1))


def test_origin_outside_raises_error():
    """Test that an error is raised if the origin is outside the geofence."""
    polygon_coords = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]
    exterior_origin = (5, 5)
    with pytest.raises(ValueError):
        Geofence(polygon_coords, exterior_origin)


def test_default_origin():
    """Test that the default origin is the center of the geofence."""
    polygon_coords = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]  # A square
    geofence = Geofence(polygon_coords)
    assert geofence.origin == (2, 2)  # Default origin should be the center
