import cv2
import picamera
import picamera.array
from dronekit import connect, VehicleMode, LocationGlobalRelative
import asyncio

# Constants
SEARCH_AREA_NW_LATITUDE = 37.790793  # Replace with the latitude of the northwest corner of the search area (decimal degrees)
SEARCH_AREA_NW_LONGITUDE = -122.401603  # Replace with the longitude of the northwest corner of the search area (decimal degrees)
SEARCH_AREA_SE_LATITUDE = 37.789793  # Replace with the latitude of the southeast corner of the search area (decimal degrees)
SEARCH_AREA_SE_LONGITUDE = -122.400603  # Replace with the longitude of the southeast corner of the search area (decimal degrees)
SEARCH_AREA_ALTITUDE = 30  # Replace with your desired altitude in meters
TARGET_RADIUS = 50  # Replace with the approximate target radius in pixels
CIRCLE_DETECTION_THRESHOLD = 100  # Adjust the threshold for circle detection sensitivity
GRID_STEP_SIZE = 20   # Replace with the step size between grid points (in meters)


# Function to capture an image from the camera asynchronously
async def capture_image_async():
    with picamera.PiCamera() as camera:
        camera.resolution = (320, 240)  # Reduce resolution for optimization
        with picamera.array.PiRGBArray(camera) as output:
            camera.capture(output, format="bgr")
            return output.array

# Function to detect circles using Hough Circle Detection asynchronously
# ... (Keep the detect_circles() function from the previous script)

# Function to detect the archery circle-like target and hotspots asynchronously
# ... (Keep the detect_target_and_hotspots() function from the previous script)

# Function for concurrent image capture, processing, and navigation with top-to-bottom lawnmower pattern
async def concurrent_top_to_bottom_lawnmower_navigation():
    # Connect to the drone
    connection_string = '/dev/ttyUSB0'  # Replace with the connection string of your drone
    vehicle = connect(connection_string, wait_ready=True)


    # Arm the drone and set to GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name == "GUIDED":
        print("Waiting for GUIDED mode...")
        await asyncio.sleep(1)

    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for arming...")
        await asyncio.sleep(1)

    # Take off to the desired altitude
    vehicle.simple_takeoff(SEARCH_AREA_ALTITUDE)
    while True:
        if vehicle.location.global_relative_frame.alt >= SEARCH_AREA_ALTITUDE * 0.95:
            print("Reached target altitude")
            break
        await asyncio.sleep(1)    

    # Create waypoints for the top-to-bottom lawnmower pattern
    waypoints = []
    lat_diff = SEARCH_AREA_SE_LATITUDE - SEARCH_AREA_NW_LATITUDE
    lon_diff = SEARCH_AREA_SE_LONGITUDE - SEARCH_AREA_NW_LONGITUDE
    lat_steps = int(lat_diff / GRID_STEP_SIZE)
    lon_steps = int(lon_diff / GRID_STEP_SIZE)

    for j in range(lon_steps):
        lon = SEARCH_AREA_NW_LONGITUDE + j * GRID_STEP_SIZE
        if j % 2 == 0:
            for i in range(lat_steps):
                lat = SEARCH_AREA_NW_LATITUDE + i * GRID_STEP_SIZE
                waypoint = LocationGlobalRelative(lat, lon, SEARCH_AREA_ALTITUDE)
                waypoints.append(waypoint)
        else:
            for i in range(lat_steps - 1, -1, -1):
                lat = SEARCH_AREA_NW_LATITUDE + i * GRID_STEP_SIZE
                waypoint = LocationGlobalRelative(lat, lon, SEARCH_AREA_ALTITUDE)
                waypoints.append(waypoint)

    # Navigate within the search area and perform target and hotspot detection
    for waypoint in waypoints:
        # Move the drone to the next waypoint in the top-to-bottom lawnmower pattern asynchronously
        await loop.run_in_executor(None, vehicle.simple_goto, waypoint)

        # Wait for the drone to reach the waypoint
        while not vehicle.location.global_relative_frame.lat - GRID_STEP_SIZE / 2 <= waypoint.lat <= vehicle.location.global_relative_frame.lat + GRID_STEP_SIZE / 2:
            await asyncio.sleep(1)

        while not vehicle.location.global_relative_frame.lon - GRID_STEP_SIZE / 2 <= waypoint.lon <= vehicle.location.global_relative_frame.lon + GRID_STEP_SIZE / 2:
            await asyncio.sleep(1)

        # Capture image from the camera asynchronously
        image = await capture_image_async()

        # Detect circles in the image using Hough Circle Detection asynchronously
        detected_circles = await loop.run_in_executor(None, detect_circles, image)

        if detected_circles is not None:
            # Process the image to classify the detected circles as target or hotspot asynchronously
            result = await loop.run_in_executor(None, detect_target_and_hotspots, image, detected_circles)

            if result is not None:
                target_type, target_x, target_y, target_radius = result
                print(f"{target_type} detected at ({target_x}, {target_y}), radius: {target_radius}")

        # Insert your logic for handling detection results and further navigation (e.g., hover, move to the next waypoint, etc.)
        # You can also add sleep time between consecutive movements to slow down the search

    # Land the drone
    vehicle.mode = VehicleMode("LAND")
    while not vehicle.mode.name == "LAND":
        await asyncio.sleep(1)

# Main function to start the concurrent top-to-bottom lawnmower navigation
if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    asyncio.run(concurrent_top_to_bottom_lawnmower_navigation())
