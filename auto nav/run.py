import drone_controller as drone
from perception import detect
import time

file =  open("drone_log.txt","a") 

flight_alt = 10
target_alt = 7
hotspot_alt = 5
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

bms_points_drift= [(12.941157663780162, 77.5659762145219),(12.941008696467064, 77.5659359665737),
             (12.940969487945662, 77.56628468686313),(12.941126330217395, 77.56629538256388)]

bms_points = [(12.941196901925672, 77.5659762285262),(12.941006073867879, 77.565949406435968),
              (12.940977344282619, 77.56625517151875),(12.941144600546245, 77.56628202178035)]

national_clg=[(12.950192443726669, 77.57267429129706),(12.949852625101393, 77.5726796557151)
              ,(12.949857824812549, 77.57303375038539),(12.950228983297505, 77.57300697670405)]

custom_locs = [(12.941105460631404, 77.56614384175194)]
# custom_locs = [(12.941134215542096, 77.5660231423459), (12.941092390216353, 77.5662538123219)]
arr=drone.grid_navigation(bms_points,6)

# vehicle = drone.connect_real_drone()

vehicle = drone.connect_simulation_drone()

vehicle.airspeed=0.5
vehicle.groundspeed=0.5

drone.arm_and_takeoff(vehicle,10)
file.write("Take off location , pos="+str(drone.get_gps_location(vehicle,0,0,0))+"\n")

detected_array = []
target_detected = False

for i in custom_locs:
    grid_point_loc = drone.LocationGlobalRelative(i[0],i[1],flight_alt)
    print("going to", grid_point_loc)
    drone.goto_wp(vehicle, grid_point_loc, ground_speed=2)
    time.sleep(2)
    
    try:
        poi = detect(vehicle, file)
        file.write("detected things -"+str(poi)+"\n")
        if len(poi) == 0:
            continue
        
        elif(len(poi) > 0):
            for points in poi:
                N,E,type = points
                if type.lower() == "target" and not target_detected:
                    gps_loc = drone.get_relative_gps_location(grid_point_loc, N, E, target_alt)

                elif type.lower() == "target" and target_detected:
                    gps_loc = drone.get_relative_gps_location(grid_point_loc, N, E, hotspot_alt)

                elif type.lower() == "hotspot":
                    gps_loc = drone.get_relative_gps_location(grid_point_loc, N, E, hotspot_alt)

                if( drone.is_not_duplicate(gps_loc, detected_array) ):
                    
                    ####### first detection of hotspot########
                    if type.lower() == "target"  and not target_detected:
                        
                        file.write("Target detected at -"+str(gps_loc)+"\n")
                        
                        drone.its_target(vehicle, gps_loc, file)
                        detected_array.append(gps_loc)
                        target_detected = True
                        file.write("target detected val changed to-"+str(target_detected)+"\n")
                        file.write("Target dropped at -"+str(drone.get_gps_location(vehicle,0,0,0))+"\n")
                
                    ###### false detection of hotspot as target##################
                    

                    if type.lower() == "target"  and  target_detected:

                        file.write("hotspot detected at -"+str(gps_loc))
                            
                        drone.its_hotspot(vehicle, gps_loc, file)
                        detected_array.append(gps_loc)

                        file.write("hotspot captured at -"+str(drone.get_gps_location(vehicle,0,0,0))+"\n")
                        file.write("false detection of above hotspot as target"+"\n")


                    ########truly detected hotspot##########

                    if type.lower() == "hotspot":

                        file.write("Hotspot detected at -"+str(gps_loc)+"\n")

                        drone.its_hotspot(vehicle,gps_loc,file)
                        detected_array.append(gps_loc)

                        file.write("Hotspot captured at-"+str(drone.get_gps_location(vehicle,0,0,0))+"\n")

                else:
                    print("duplicate detected .......skipping............")
    except Exception as e:
        print("error occured during detection-"+str(e))
        file.write("error occured during detection-"+str(e)+"\n")
        continue
    

drone.RTL(vehicle)

