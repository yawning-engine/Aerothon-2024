import drone_controller as drone
import time

f= open("test_log.txt",'a')
vehicle = drone.connect_real_drone()

vehicle.airspeed=0.5
vehicle.groundspeed=0.5

drone.arm_and_takeoff(vehicle,5)
f.write("Take off successful")
lat = 12.950108655161891 
lon = 77.57291198864498




f.write("going to wp lat ,lon "+str( lat)+str( lon))
drone.goto_wp(vehicle,drone.LocationGlobalRelative(lat,lon,10),ground_speed=1)

f.write("changing mode to LAND")
drone.RTL(vehicle)
f.write("Mode changed to LAND")
