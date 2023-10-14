

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative


# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


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
        print(" Waiting for mode change ...")
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


arm_and_takeoff(10)
vehicle.airspeed=30
vehicle.groundspeed=30

point1 = LocationGlobalRelative(28.3670466,77.3166490, 30)
vehicle.simple_goto(point1)
time.sleep(25)
print("changeing alt to 10")
point1 = LocationGlobalRelative(28.3670466,77.3166490, 10)
vehicle.simple_goto(point1)
print("changed alt to 10")
print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")
while not vehicle.mode.name=="RTL":
    print("Waiting to switch mode to RTL, current mode:",vehicle.mode.name)
    time.sleep(1)
    
# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

