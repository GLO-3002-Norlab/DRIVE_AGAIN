from shapely.geometry import Point, Polygon

def create_polygon(coordinates):
    """
    Create a polygon from a list of (x, y) coordinates.
    :param coordinates: List of tuples representing the polygon's vertices.
    :return: A shapely Polygon object.
    """
    return Polygon(coordinates)

def is_point_inside(polygon, point):
    """
    Check if a point is inside the given polygon.
    :param polygon: A shapely Polygon object.
    :param point: A tuple (x, y) representing the point.
    :return: True if the point is inside, False otherwise.
    """
    return polygon.contains(Point(point))

if __name__ == "__main__":
    polygon_coords = [(0, 0), (4, 0), (4, 4), (0, 4)]  # A square
    polygon = create_polygon(polygon_coords)
    
    test_points = [(2, 2), (5, 5), (3, 3), (-1, -1)]
    
    for point in test_points:
        result = is_point_inside(polygon, point)
        print(f"Point {point} is inside the polygon: {result}")