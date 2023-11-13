import drone_controller as drone
import time
# field 1-
# dronekit-sitl copter --home=13.394622,77.731250,0,180

# filed 2-
# dronekit-sitl copter --home=13.394700,77.731850,0,180



file =  open("drone_log.txt","a") 


vehicle = drone.connect_dorne()

boundries=[drone.LocationGlobalRelative(13.394727,77.7311024,30),
     drone.LocationGlobalRelative(13.393622,77.7309197,30),
     drone.LocationGlobalRelative(13.39352,77.7313704,30),
     drone.LocationGlobalRelative(13.394631,77.7315509,30)]

field_1_boundary_points = [(13.394727, 77.7311024), (13.393622, 77.7309197), 
                           (13.39352, 77.7313704), (13.394631, 77.7315509)]

field_2_boundary_points = [(13.394622, 77.7316004), (13.393506, 77.7314371),
                           (13.393385, 77.7318823), (13.39454, 77.7320772)]
bms_points= [(12.94118941450555, 77.56595421106643),(12.941040411783515, 77.56596493990145),
             (12.940995972357955, 77.56626266507376),(12.941137132858843, 77.56631362704019)]

arr=drone.grid_navigation(bms_points,5)

print(arr)

vehicle.airspeed=1
vehicle.groundspeed=1


# file.write("Take off location , pos="+drone.get_gps_location(vehicle,0,0,0))

# drone.arm_and_takeoff(vehicle,15)

# file.write("Take off successfull , pos="+str(drone.get_gps_location(vehicle,0,0,0)))

print("relatve to abs :"+str(drone.get_gps_location(vehicle,5,5,15)))
print(arr)
target_count = 0
hotspot_count = 0

detected_array = []

for i in arr:
    print("going to"+str(drone.LocationGlobalRelative(i[0],i[1],15)))

    drone.goto_wp(vehicle, drone.LocationGlobalRelative(i[0],i[1],15),ground_speed=1)
    file.write("reached wp-"+str(i))
    # time.sleep(2)
    # poi= 
    # if len(poi)==0:
    #     continue
    
    # elif(len(poi)>0):
    #     for points in poi:
    #         N,E,type = points
    #         gps_loc = drone.get_gps_location(vehicle,N,E)

    #         if( drone.is_not_duplicate(gps_loc,detected_array) ):

    #             if type.lower() == "target" :

    #                 file.write("Target detected at -",gps_loc)
    #                 detected_array.append(gps_loc)
    #                 drone.its_target(vehicle,N,E)
    #                 file.write("Target dropped at -",drone.get_gps_location(vehicle,0,0,0))

    #             if type.lower() == "hotspot":

    #                 file.write("Hotspot detected at -", gps_loc)
    #                 detected_array.append(gps_loc)
    #                 drone.its_hotspot(vehicle,N,E)
    #                 file.write("Hotspot captured at-",drone.get_gps_location(vehicle,0,0,0))

        

    # if not drone.get_gps_location(vehicle,10,10) in hotspot_arr:
    #     drone.its_hotspot(vehicle,10,10)
    #     hotspot_arr.append(drone.get_gps_location(vehicle,10,10))


    # if not drone.get_gps_location(vehicle,10,10) in target_arr:
    #     drone.its_target(vehicle,10,10)
    #     target_arr.append(drone.get_gps_location(vehicle,10,10))

# file.write("Returning to home")

# drone.RTL(vehicle)

# file.write("hurayy, Mission successful")
vehicle.close()