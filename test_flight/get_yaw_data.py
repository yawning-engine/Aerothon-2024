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

flt_alt = 10
drop_alt = 5

ground_speed = 0.5


def connectmycopter():

    connection_string = "/dev/serial0"
    baud_rate = 912600
    print("Connecting to drone...")
    #f.write("\n Connecting to drone...")
    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    return vehicle

def get_yaw(vehicle,f):

    print("Attitude:",vehicle.attitude)
    print("YAW:",vehicle.attitude.yaw)
    
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


vehicle = connectmycopter()

f = open("log_get_yaw_data.txt", 'w')

basic_data(vehicle, f)
get_yaw(vehicle,f)

print("\n--------Mission Successfull--------\n")
f.write("\n--------Mission Successfull--------")

vehicle.armed = False
vehicle.close()

