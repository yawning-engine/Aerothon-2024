

from __future__ import print_function
import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil


# Set up option parsing to get connection string

import dronekit_sitl

f = open("drone_log.txt", 'w')
sitl = None
def connect_dorne(connection_string):
    
    

    # Start SITL if no connection string specified
    if not connection_string:
        
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()


    # Connect to the Vehicle
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)
    return vehicle
def distance_to_wp(vehicle , wp):

    dist = 0.0

    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon
    alt = vehicle.location.global_relative_frame.alt

    latdiff = wp.lat - lat
    londiff = wp.lon - lon
    altdiff = wp.alt - alt

    dist = math.sqrt((latdiff * latdiff) + (londiff * londiff)) * 1.113195e5

    return dist

def goto_wp(vehicle, wp, f=f,ground_speed=3):
 
    
    
    print("Going to waypoint")
    f.write("\n\nGoing to waypoint:%s"%str(wp))
    vehicle.groundspeed = ground_speed
    print("\nGround Speed:%f"%vehicle.groundspeed)
    f.write("\nGround Speed:%f\n"%vehicle.groundspeed)
    if distance_to_wp(vehicle, wp)>40:
        ground_speed=10
    vehicle.simple_goto(wp, groundspeed=ground_speed)
    
    while distance_to_wp(vehicle, wp) > 1:
        dist = distance_to_wp(vehicle, wp)
        if dist > 0.05:
            print("Distance to waypoint:%f" % dist)
            f.write("\nDistance to waypoint:%f" % dist)
        else:
            print("Waypoint %s Reached"%str(wp))
            f.write("\nWaypoint %s Reached"%str(wp))
            break
        time.sleep(0.3)
    condition_yaw(vehicle)
    return None


def get_gps_location(vehicle, dNorth, dEast , alt=30, f=f):

    """
    goes to relative loc, input param in meters
    """

    
    earth_radius = 6378137.0  # Radius of "spherical" earth
    lat_now = vehicle.location.global_relative_frame.lat
    lon_now = vehicle.location.global_relative_frame.lon
    alt_now = vehicle.location.global_relative_frame.alt
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * lat_now / 180))

    # New position in decimal degrees
    newlat = lat_now + (dLat * 180 / math.pi)
    newlon = lon_now + (dLon * 180 / math.pi)

    if type(vehicle.location.global_relative_frame) is LocationGlobalRelative:
        targetlocation = LocationGlobalRelative(newlat, newlon, alt)
    else:
        raise Exception("Invalid Location object passed")

    print("New location coordinates: %s"%str(targetlocation))
    f.write("\nNew location coordinates: %s"%str(targetlocation))
    print("target format",targetlocation)
    return targetlocation


def condition_yaw(vehicle, heading=180, f=f, relative=False):
    """
    This function yaws the drone towards north
    """

    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle

    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    print("Current Heading: %s"%str(vehicle.attitude.yaw))
    f.write("\nCurrent Heading: %s" % str(vehicle.attitude.yaw))
    time.sleep(1)
    print("Yawing NORTH")
    f.write("\n\nYawing NORTH")
    print("Current Heading: %s"%str(vehicle.attitude.yaw))
    f.write("\nCurrent Heading: %s" % str(vehicle.attitude.yaw))
    vehicle.send_mavlink(msg)

def arm_and_takeoff(vehicle,aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.

    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    


    print("Arming motors")
    # Copter should arm in GUIDED mode
    print("\nSet Vehicle.mode = GUIDED (currently: %s)" % vehicle.mode.name) 
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name=='GUIDED':  #Wait until mode has changed
        
        print("Waiting to switch mode to GUIDED, current mode:",vehicle.mode.name)
    time.sleep(1)
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)   


    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def take_screenshot():
    return 0

def drop():
    return 1

def is_not_duplicate(curr_point, arr):
    for i in arr:
        if i in arr:
            return 1

def its_hotspot(vehicle,x,y):
    """
    x: relative north in meters
    y: relative East in meters
    """
    print("hotspot mission started....")
    abs_loc=get_gps_location(vehicle,x,y,30)
    goto_wp(vehicle, abs_loc)
    time.sleep(2)
    take_screenshot()
    

def its_target(vehicle,x,y):
    '''
    x: relative north in meters
    y: relative East in meters
    '''
    print("target mission started....")
    goto_wp(vehicle, get_gps_location(vehicle,x,y,20))
    time.sleep(2)
    drop()
    print("target mission successful")


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

def grid_navigation(points, max_distance):
    if len(points) != 4:
        raise ValueError("You must provide exactly 4 GPS points.")
    # min_lon = points[0][1]
    # min_lat = points[0][0]
    # max_lon = points[-1][1]
    # max_lat = points[1][0]

    
    # lat_range = calculate_distance((min_lat, min_lon), (max_lat, min_lon))
    # lon_range = calculate_distance((min_lat, min_lon), (min_lat, max_lon))
    # Sort points to form a rectangular area
    min_lat= points[0][0]
    min_lon = points[1][1]
    max_lat = points[1][0]
    max_lon = points[2][1]
    # Calculate the number of grid points needed in each dimension
    
    lat_range = calculate_distance((min_lat, min_lon), (max_lat, min_lon))
    lon_range = calculate_distance((min_lat, min_lon), (min_lat, max_lon))
    print("range -", lat_range,lon_range)
    num_lat_points = round(int(lat_range / max_distance)) + 1
    num_lon_points = round(int(lon_range / max_distance)) + 1
    
    points.sort(key=lambda p: (p[0], p[1]))  # Sort by latitude and then longitude
    min_lat, min_lon = points[0]
    max_lat, max_lon = points[-1]
    
    # Generate the grid points
    grid_points = []
    lat_step = (max_lat - min_lat) / num_lat_points
    lon_step = (max_lon - min_lon) / num_lon_points
    # lon angle depens on variation in lon and vise versa
    lon_angle = points[0][1]-points[3][1]
    lat_angle = points[1][0]-points[0][0]
    print("lat_angle",lat_angle,lon_angle)
    # min_lat, min_lon = points[0]
    # max_lat, max_lon = points[-1]
    line_arr=[]
    x=1
    for i in range(num_lat_points):
        line_arr=[]
        x*=-1
        for j in range(num_lon_points+1):

            print("lat step change-",i*lat_angle/num_lat_points)
            print("lon step change=", (j-1)*lon_angle/num_lon_points)

            if j==0:
                lat = min_lat + i * lat_step + (j)*lat_angle/(num_lon_points+1)
                lon = min_lon + 0.5 * lon_step + i*lon_angle/(num_lat_points+1)
            else:
                lat = min_lat + i * lat_step + (j)*lat_angle/(num_lon_points+1)
                lon = min_lon + j * lon_step + 0.5 * lon_step +i*lon_angle/(num_lat_points+1)
            # lat = min_lat + i * lat_step 
            # lon = min_lon + j * lon_step


            line_arr.append((lat, lon,30))
        if x==1:
            for i in range(len(line_arr)-1,-1,-1):
                grid_points.append(line_arr[i])
        else:
            for i in range(len(line_arr)):
                grid_points.append(line_arr[i])



    # Ensure that the grid points do not exceed the boundary points
    # grid_points = [point for point in grid_points if min_lat <= point[0] <= max_lat and min_lon <= point[1] <= max_lon]
    print("grid points=",grid_points)
    return grid_points

# Example usage:


# grid_points now contains a list of GPS points forming a grid within the specified boundary.


def RTL(vehicle):
    print("changed alt to 10")
    print("Returning to Launch")
    vehicle.VehicleMode("RTL")
    while not vehicle.mode.name=="RTL":
        print("Waiting to switch mode to RTL, current mode:",vehicle.mode.name)
        time.sleep(1)
        
    # Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()