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


def connectmycopter():

    connection_string = "/dev/serial0"
    baud_rate = 57600
    print("Connecting to drone...")
    f.write("\n Connecting to drone...")
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


def actuate_servo(vehicle, action, f):
    
    if action == "open":
        msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 10, 1000, 0, 0, 0, 0, 0)
        print("\n\nPayload Dropped\n\n")
        f.write("\n\nPayload Dropped\n\n")
        print("drop at:",vehicle.location.global_relative_frame)	
        f.write("drop at:" + str(vehicle.location.global_relative_frame))
        time.sleep(0.5)

    if action == "close":
        msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 10, 2000, 0, 0, 0, 0, 0)
        print("\n\nPayload Secured\n\n")
        f.write("\n\nPayload Secured\n\n")
        print("drop at:",vehicle.location.global_relative_frame)	
        f.write("drop at:" + str(vehicle.location.global_relative_frame))
        time.sleep(0.5)

    vehicle.send_mavlink(msg)

    return None


f = open("log_payload_drop.txt", 'w')

vehicle = connectmycopter()

basic_data(vehicle, f)

actuate_servo(vehicle,"open",f)
time.sleep(3)
actuate_servo(vehicle,"close",f)
time.sleep(3) 
actuate_servo(vehicle,"open",f)time.sleep(3)
print("\n--------Mission Successfull--------\n")
f.write("\n--------Mission Successfull--------")

vehicle.armed = False
vehicle.close()

