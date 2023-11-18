import drone_controller as drone
# from perception import detect
import time

f =  open("drone_log.txt","w") 

flight_alt = 30
target_alt = 20
hotspot_alt = 10
# field 1-
# dronekit-sitl copter --home=13.394622,77.731250,0,180

# filed 2-
# dronekit-sitl copter --home=13.394700,77.731850,0,180

# national clg -
# dronekit-sitl copter --home=12.950189431098176, 77.5729052662308,0,0


field_2_boundary_points = [(13.394727, 77.7311024), (13.393622, 77.7309197), 
                           (13.39352, 77.7313704), (13.394631, 77.7315509)]

field_1_boundary_points = [(13.394622, 77.7316004), (13.393506, 77.7314371),
                           (13.393385, 77.7318823), (13.39454, 77.7320772)]

custom_loc = [(13.410502262701508, 77.69841723282588)]
arr=drone.grid_navigation(field_1_boundary_points,20)

vehicle = drone.connect_simulation_drone()

# vehicle = drone.connect_real_drone()

vehicle.airspeed = 0.5
vehicle.groundspeed = 0.5

drone.arm_and_takeoff(vehicle,20)
f.write("Take off location , pos="+str(drone.get_gps_location(vehicle,0,0,0))+"\n")
drone.arm_and_takeoff(vehicle,20)
f.write("Take off location , pos="+str(drone.get_gps_location(vehicle,0,0,0))+"\n")

detected_array = []
target_detected = False

for i in arr:
    grid_point_loc = drone.LocationGlobalRelative(i[0],i[1],flight_alt)
    print("going to", grid_point_loc)
    drone.goto_wp(vehicle, grid_point_loc, ground_speed=3)
    time.sleep(1)
    '''
    try:
        poi = detect(vehicle, f)
        f.write("detected things -"+str(poi)+"\n")
        print("detected")
        if len(poi) == 0:
            continue
        
        elif(len(poi) > 0):
            for points in poi:
                E,N,type = points
                if type.lower() == "target" and not target_detected:
                    gps_loc = drone.get_relative_gps_location(grid_point_loc, N, E, target_alt)

                elif type.lower() == "target" and target_detected:
                    gps_loc = drone.get_relative_gps_location(grid_point_loc, N, E, hotspot_alt)

                elif type.lower() == "hotspot":
                    gps_loc = drone.get_relative_gps_location(grid_point_loc, N, E, hotspot_alt)

                if( drone.is_not_duplicate(gps_loc, detected_array) ):
                    
                    ####### first detection of hotspot########
                    if type.lower() == "target"  and not target_detected:
                        
                        f.write("Target detected at -"+str(gps_loc)+"\n")
                        
                        drone.its_target(vehicle, gps_loc, f)
                        detected_array.append(gps_loc)
                        target_detected = True
                        f.write("target detected val changed to-"+str(target_detected)+"\n")
                        f.write("Target dropped at -"+str(drone.get_gps_location(vehicle,0,0,0))+"\n")
                
                    ###### false detection of hotspot as target##################
                    

                    if type.lower() == "target"  and  target_detected:

                        f.write("hotspot detected at -"+str(gps_loc))
                            
                        drone.its_hotspot(vehicle, gps_loc, f)
                        detected_array.append(gps_loc)

                        f.write("hotspot captured at -"+str(drone.get_gps_location(vehicle,0,0,0))+"\n")
                        f.write("false detection of above hotspot as target"+"\n")


                    ########truly detected hotspot##########

                    if type.lower() == "hotspot":

                        f.write("Hotspot detected at -"+str(gps_loc)+"\n")

                        drone.its_hotspot(vehicle,gps_loc,f)
                        detected_array.append(gps_loc)

                        f.write("Hotspot captured at-"+str(drone.get_gps_location(vehicle,0,0,0))+"\n")

                else:
                    print("duplicate detected .......skipping............")
    except Exception as e:
        print("error occured during detection-"+str(e))
        f.write("error occured during detection-"+str(e)+"\n")
        continue
    '''

drone.RTL(vehicle)

