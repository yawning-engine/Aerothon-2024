

from __future__ import print_function
import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil


# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None
ground_speed = 10

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
f = open("drone_log.txt", 'w')

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

def goto_wp(vehicle, wp, f=f):
 
    global ground_speed
    
    print("Going to waypoint")
    f.write("\n\nGoing to waypoint:%s"%str(wp))
    vehicle.groundspeed = ground_speed
    print("\nGround Speed:%f"%vehicle.groundspeed)
    f.write("\nGround Speed:%f\n"%vehicle.groundspeed)

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
    condition_yaw()
    return None


def get_location_metres(vehicle, dNorth, dEast , alt=30, f=f):

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


def condition_yaw(vehicle=vehicle, heading=180, f=f, relative=False):
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

def arm_and_takeoff(aTargetAltitude):
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

def its_hotspot(x,y):
    """
    x: relative north in meters
    y: relative East in meters
    """
    print("hotspot mission started....")
    abs_loc=get_location_metres(vehicle,x,y,30)
    if not abs_loc in hotspot_arr:
        goto_wp(vehicle, abs_loc)
        time.sleep(2)
        take_screenshot()
    else:
        print("duplicate, skipping")
        print("hotspot mission successful")
    

def its_target(x,y):
    '''
    x: relative north in meters
    y: relative East in meters
    '''
    print("target mission started....")
    goto_wp(vehicle, get_location_metres(vehicle,x,y,20))
    time.sleep(2)
    drop()
    print("target mission successful")


vehicle.airspeed=30
vehicle.groundspeed=30

arm_and_takeoff(10)

condition_yaw()
print("relatve to abs :",get_location_metres(vehicle,5,5,20))

hotspot_arr=[get_location_metres(vehicle,10,10)]
target_arr=[get_location_metres(vehicle,10,-10)]
arr=[LocationGlobalRelative(-35.3629941,149.1677928,30),LocationGlobalRelative(-35.3628410,149.1706145,30),LocationGlobalRelative(-35.3629941,149.1677928,30),LocationGlobalRelative(-35.3628410,149.1706145,30)]

for i in arr:
    print("going to", i)
    goto_wp(vehicle, i)
    its_hotspot(10,10)
    its_target(15,-10)

print("changed alt to 10")
print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")
while not vehicle.mode.name=="RTL":
    print("Waiting to switch mode to RTL, current mode:",vehicle.mode.name)
    time.sleep(1)
    
# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

