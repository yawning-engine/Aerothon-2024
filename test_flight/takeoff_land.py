from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import cv2
import socket
import argparse
import dronekit_sitl
import pymavlink
from pymavlink import mavutil
import numpy as np

x=0
y=0

flt_alt = 5
drop_alt = 5

ground_speed = 0.5


def connectmycopter():

    '''
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    connection_string = args.connect
    '''
    connection_string = "/dev/serial0"
    baud_rate = 57600
    print("Connecting to drone...")
    #f.write("\n Connecting to drone...")
    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    return vehicle


def basic_data(vehicle, f):

    print("Version: %s" % vehicle.version)
    print("Armable: %s" % vehicle.is_armable)
    print("Vehicle Mode: %s" % vehicle.mode.name)
    print("Support set attitude from companion: %s" % vehicle.capabilities.set_attitude_target_local_ned)
    print("Last Heartbeat: %s" % vehicle.last_heartbeat)
    f.write("Version: %s" % vehicle.version)
    f.write("\nArmable: %s" % vehicle.is_armable)
    f.write("\nVehicle Mode: %s" % vehicle.mode.name)
    f.write("\nSupport set attitude from companion: %s" % vehicle.capabilities.set_attitude_target_local_ned)
    f.write("\nLast Heartbeat: %s\n" % vehicle.last_heartbeat)
    
    
def get_position(vehicle, f):

    print("\nPosition: %s" % vehicle.location.global_relative_frame)
    f.write("\nPosition: %s" % vehicle.location.global_relative_frame)
    return None


def set_home(vehicle, f):

    global flt_alt
    
    home = vehicle.location.global_relative_frame
    home.alt = flt_alt
    
    print("\nHome set at: %s" % vehicle.location.global_relative_frame)
    f.write("\nHome set at: %s" % vehicle.location.global_relative_frame)
    
    return home


def arm_and_takeoff(targetaltitude, vehicle, f):

    while vehicle.is_armable == False:
        print("Waiting for vehicle to become armable")
        f.write("Waiting for vehicle to become armable")
        time.sleep(1)

    vehicle.mode = VehicleMode("STABILIZE")
    print("Current mode: %s" % vehicle.mode.name)
    f.write("\nCurrent mode: %s" % vehicle.mode.name)

    vehicle.armed = True
    while vehicle.armed == False:
        print("Waiting for vehicle to arm")
        f.write("\nWaiting for vehicle to arm")
        time.sleep(1)

    print("VEHICLE ARMED")
    f.write("\n\nVEHICLE ARMED\n")
    print("About to takeoff")
    f.write("\nAbout to takeoff")

    time.sleep(2)

    vehicle.mode = VehicleMode("GUIDED")
    print("\n\nGuided mode\n\n")
    f.write("\n\nGUIDED MODE\n\n")
    	
    while vehicle.mode != "GUIDED":
        print("Waiting for vehicle to enter guided mode")
        f.write("\nWaiting for vehicle to enter guided mode")
        time.sleep(0.3)
    
    time.sleep(3)
    vehicle.simple_takeoff(targetaltitude)

    while True:
        alt_now = vehicle.location.global_relative_frame.alt
        print("Altitude:%f" % alt_now)
        f.write("\nAltitude:%f" % alt_now)
        if alt_now >= flt_alt * 0.98:
            break
        time.sleep(0.3)

    time.sleep(2)
    print("Target altitude reached")
    f.write("\nTarget altitude reached")

    return None


def land_copter(vehicle, f):
	
	vehicle.mode = VehicleMode("LAND")
	print("\n\nLand Mode\n\n")
	f.write("\n\nLand Mode\n\n")
	
	while True:
		alt_now = vehicle.location.global_relative_frame.alt
		print("Altitude:%f"%alt_now)
		f.write("\nAltitude:%f"%alt_now)
		if alt_now <= 0.15:
			break
		time.sleep(0.3)
	
	time.sleep(3)

	print("Landed successfully")
	f.write("\nLanded Successfully\n")

	return None
	
	
vehicle = connectmycopter()

f = open("log_takeoff_land.txt", 'w')

basic_data(vehicle, f)

home_wp = set_home(vehicle, f)
time.sleep(2)

arm_and_takeoff(flt_alt, vehicle, f)
time.sleep(2)

land_copter(vehicle, f)
time.sleep(3)

print("\n--------Mission Successfull--------\n")
f.write("\n--------Mission Successfull--------")

vehicle.armed = False
vehicle.close()

