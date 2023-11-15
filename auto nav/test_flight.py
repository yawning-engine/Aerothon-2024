import drone_controller as drone
import time

f= open("test_log.txt",'a')
vehicle = drone.connect_real_drone()

drone.arm_and_takeoff(vehicle,5)
f.write("Take off successful")
lat = 12.941942516782559 
lon = 77.56657882884068

# f.write("going to wp lat ,lon ", lat, lon)
# drone.goto_wp(vehicle,drone.LocationGlobalRelative(lat,lon,5),0.5)

f.write("changing mode to LAND")
drone.Land(vehicle)
f.write("Mode changed to LAND")
