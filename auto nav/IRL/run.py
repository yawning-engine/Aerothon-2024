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
bms_points= [(12.941209902836812, 77.5659708787648),(12.941037373375236, 77.56592259900712),
             (12.94096417902213, 77.56628737939842),(12.9411497789471, 77.56631688369477)]

national_clg=[(12.9509975088598, 77.57268620201272),(12.950826293163624, 77.57268351980396)
              ,(12.95079884636169, 77.57291016644415),(12.950951764219674, 77.57290614313101)]

arr=drone.grid_navigation(national_clg,10)

print(arr)

vehicle.airspeed=0.5
vehicle.groundspeed=0.5


file.write("Take off location , pos="+str(drone.get_gps_location(vehicle,0,0,0)))

drone.arm_and_takeoff(vehicle,10)
time.sleep(2)

file.write("Take off successfull , pos="+str(drone.get_gps_location(vehicle,0,0,0)))

print("relatve to abs :"+str(drone.get_gps_location(vehicle,5,5,15)))
print(arr)
target_count = 0
hotspot_count = 0

detected_array = []

for i in arr:
    print("going to"+str(drone.LocationGlobalRelative(i[0],i[1],15)))

    drone.goto_wp(vehicle, drone.LocationGlobalRelative(i[0],i[1],15),ground_speed=0.5)
    file.write("reached wp-"+str(i))
    time.sleep(2)
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
file.write("Returning to home")

drone.RTL(vehicle)

file.write("hurayy, Mission successful")
vehicle.close()
