import math

def calculate_distance(point1, point2):
    # Calculate the Euclidean distance between two GPS points.
    lat1, lon1 = point1
    lat2, lon2 = point2
    earth_radius = 6371000  # Earth's radius in meters
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) * math.sin(dlat / 2) + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) * math.sin(dlon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = earth_radius * c
    return distance

def grid_navigation(points, max_distance=25):
    if len(points) != 4:
        raise ValueError("You must provide exactly 4 GPS points.")

    # Sort points to form a rectangular area
    points.sort(key=lambda p: (p[0], p[1]))  # Sort by latitude and then longitude

    # Calculate the number of grid points needed in each dimension
    min_lat, min_lon = points[0]
    max_lat, max_lon = points[-1]
    lat_range = calculate_distance((min_lat, min_lon), (max_lat, min_lon))
    lon_range = calculate_distance((min_lat, min_lon), (min_lat, max_lon))
    print("range=",lat_range,lon_range)
    num_lat_points = int(lat_range / max_distance) + 1
    num_lon_points = int(lon_range / max_distance) + 1
    print("num of points",num_lat_points,num_lon_points)
    # Generate the grid points
    grid_points = []
    lat_step = (max_lat - min_lat) / num_lat_points
    lon_step = (max_lon - min_lon) / num_lon_points
    print("steps=",lat_step,lon_step)
    for i in range(1,num_lat_points+1):
        for j in range(1,num_lon_points+1):
            
            lat = min_lat + i * lat_step
            lon = min_lon + j * lon_step
            #print("point",i,j,"=",lat,lon)
            grid_points.append((lat, lon))

    # Ensure that the grid points do not exceed the boundary points
    # grid_points = [point for point in grid_points if min_lat <= point[0] <= max_lat and min_lon <= point[1] <= max_lon]

    return grid_points

# Example usage:
boundary_points = [(13.394622, 77.7316004), (13.393506, 77.7314371), (13.393385, 77.7318823), (13.39454, 77.7320772)]

grid_points = grid_navigation(boundary_points, max_distance=10)
#print(grid_points)
# grid_points now contains a list of GPS points forming a grid within the specified boundary.
