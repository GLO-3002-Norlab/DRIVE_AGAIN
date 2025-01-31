from shapely.geometry import Point, Polygon

class Geofence:
    def __init__(self, coordinates):
        """
        Initialize the Geofence with a list of (x, y) coordinates.
        :param coordinates: List of tuples representing the polygon's vertices.
        """
        self.polygon = Polygon(coordinates)

    def is_point_inside(self, point):
        """
        Check if a point is inside the geofence.
        :param point: A tuple (x, y) representing the point.
        :return: True if the point is inside, False otherwise.
        """
        return self.polygon.contains(Point(point))

if __name__ == "__main__":
    polygon_coords = [(0, 0), (4, 0), (4, 4), (0, 4)]  # A square
    geofence = Geofence(polygon_coords)
    
    test_points = [(2, 2), (5, 5), (3, 3), (-1, -1)]
    
    for point in test_points:
        result = geofence.is_point_inside(point)
        print(f"Point {point} is inside the polygon: {result}")