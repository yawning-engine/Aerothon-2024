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
vehicle = drone.connect_dorne(connection_string)

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
bms_points= [(12.94135901438728, 77.56516586950416),(12.940433628249703, 77.56571304009111),
             (12.941094992138853, 77.5665069738839),(12.94186353137158, 77.56604026955976)]

arr=drone.grid_navigation(field_2_boundary_points,15)

print(arr)

vehicle.airspeed=2
vehicle.groundspeed=2

drone.arm_and_takeoff(vehicle,10)

#drone.condition_yaw()
print("relatve to abs :",drone.get_gps_location(vehicle,5,5,20))
print(arr)
for i in arr:
    print("going to", drone.LocationGlobalRelative(i[0],i[1],i[2]))
    drone.goto_wp(vehicle, drone.LocationGlobalRelative(i[0],i[1],i[2]))


    # if not drone.get_gps_location(vehicle,10,10) in hotspot_arr:
    #     drone.its_hotspot(vehicle,10,10)
    #     hotspot_arr.append(drone.get_gps_location(vehicle,10,10))


    # if not drone.get_gps_location(vehicle,10,10) in target_arr:
    #     drone.its_target(vehicle,10,10)
    #     target_arr.append(drone.get_gps_location(vehicle,10,10))

drone.RTL(vehicle)

