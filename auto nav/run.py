import drone_controller as drone
# from perception import detect
import time

file =  open("drone_log.txt","a") 

flight_alt =15
target_alt = 10

# field 1-
# dronekit-sitl copter --home=13.394622,77.731250,0,180

# filed 2-
# dronekit-sitl copter --home=13.394700,77.731850,0,180

# national clg -
# dronekit-sitl copter --home=12.950189431098176, 77.5729052662308,0,0


field_1_boundary_points = [(13.394727, 77.7311024), (13.393622, 77.7309197), 
                           (13.39352, 77.7313704), (13.394631, 77.7315509)]

field_2_boundary_points = [(13.394622, 77.7316004), (13.393506, 77.7314371),
                           (13.393385, 77.7318823), (13.39454, 77.7320772)]

bms_points= [(12.941209902836812, 77.5659708787648),(12.941037373375236, 77.56592259900712),
             (12.94096417902213, 77.56628737939842),(12.9411497789471, 77.56631688369477)]

national_clg=[(12.9509975088598, 77.57268620201272),(12.950826293163624, 77.57268351980396)
              ,(12.95079884636169, 77.57291016644415),(12.950951764219674, 77.57290614313101)]


arr=drone.grid_navigation(national_clg,10)

vehicle = drone.connect_simulation_drone()
vehicle.airspeed=0.5
vehicle.groundspeed=0.5

drone.arm_and_takeoff(vehicle,15)
file.write("Take off location , pos="+str(drone.get_gps_location(vehicle,0,0,0)))

detected_array = []

for i in arr:
    grid_point_loc = drone.LocationGlobalRelative(i[0],i[1],flight_alt)
    print("going to", grid_point_loc)
    drone.goto_wp(vehicle, grid_point_loc, ground_speed=5)
    '''
    poi = detect(vehicle, )
    if len(poi)==0:
        continue
    
    elif(len(poi)>0):
        for points in poi:
            N,E,type = points
            gps_loc = drone.get_relative_gps_location(grid_point_loc, N, E, target_alt)

            if( drone.is_not_duplicate(gps_loc, detected_array) ):

                if type.lower() == "target" :

                    file.write("Target detected at -"+str(gps_loc))
                    
                    drone.its_target(vehicle, gps_loc, file)
                    detected_array.append(gps_loc)

                    file.write("Target dropped at -"+str(drone.get_gps_location(vehicle,0,0,0)))

                if type.lower() == "hotspot":

                    file.write("Hotspot detected at -"+str(gps_loc))

                    drone.its_hotspot(vehicle,gps_loc,file)
                    detected_array.append(gps_loc)

                    file.write("Hotspot captured at-"+str(drone.get_gps_location(vehicle,0,0,0)))

            else:
                print("duplicate detected .......skipping............")
    '''
drone.RTL(vehicle)

