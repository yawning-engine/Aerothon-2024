import drone_controller as drone
import argparse
import time
# field 1-
# dronekit-sitl copter --home=13.394622,77.731250,0,180

# filed 2-
# dronekit-sitl copter --home=13.394700,77.731850,0,180

parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                        help='''Vehicle connection target string. If not specified, 
                        SITL automatically started and used.''')
args = parser.parse_args()

connection_string = args.connect
vehicle = drone.connect_simulation_dorne(connection_string)

hotspot_arr=[drone.get_gps_location(vehicle,10,10)]
target_arr=[drone.get_gps_location(vehicle,10,-10)]
boundries=[drone.LocationGlobalRelative(13.394727,77.7311024,30),
     drone.LocationGlobalRelative(13.393622,77.7309197,30),
     drone.LocationGlobalRelative(13.39352,77.7313704,30),
     drone.LocationGlobalRelative(13.394631,77.7315509,30)]

field_1_boundary_points = [(13.394727, 77.7311024), (13.393622, 77.7309197), 
                           (13.39352, 77.7313704), (13.394631, 77.7315509)]

field_2_boundary_points = [(13.394622, 77.7316004), (13.393506, 77.7314371),
                           (13.393385, 77.7318823), (13.39454, 77.7320772)]
bms_points= [(12.941147574684594, 77.56620096320266),(12.941035238094432, 77.56620366070055),
             (12.941024743662918, 77.56635390367906),(12.941158056317148, 77.56634852137806)]

arr=drone.grid_navigation(field_1_boundary_points,15)

#print(arr)

vehicle.airspeed=2
vehicle.groundspeed=2

file =  open("drone_log.txt","a") 
#drone.arm_and_takeoff(vehicle,30)
file.write("Take off location , pos="+str(drone.get_gps_location(vehicle,0,0,0)))

#drone.condition_yaw()
print("relatve to abs :"+str(drone.get_gps_location(vehicle,5,5,15)))
#print(arr)
for i in arr:
    print("going to", drone.LocationGlobalRelative(i[0],i[1],15))
    drone.goto_wp(vehicle, drone.LocationGlobalRelative(i[0],i[1],15),ground_speed=5)



    # if not drone.get_gps_location(vehicle,10,10) in hotspot_arr:
    #     drone.its_hotspot(vehicle,10,10)
    #     hotspot_arr.append(drone.get_gps_location(vehicle,10,10))


    # if not drone.get_gps_location(vehicle,10,10) in target_arr:
    #     drone.its_target(vehicle,10,10)
    #     target_arr.append(drone.get_gps_location(vehicle,10,10))

drone.RTL(vehicle)

