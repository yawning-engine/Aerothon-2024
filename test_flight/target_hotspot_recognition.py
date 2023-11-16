# import the necessary packages
from dronekit import connect, VehicleMode, LocationGlobalRelative
import socket
import argparse
import dronekit_sitl
import pymavlink
from pymavlink import mavutil
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
import numpy as np
import matplotlib.pyplot as plt
import time
import math

# Initialize flight variables
flt_alt = 15
drop_alt = 5

ground_speed = 0.5

yaw_angle = 0

# Initialize Picam capture
width = 640
height = 480
center_cord = [int(width/2),int(height/2)]

    
# Load the template image of the target and hotspot
template_path_target1 = 'target_real_smol.png'  # Change this to your target image's path
template_path_target2 = 'target_real_full.png'  # Change this to your target image's path

template_target1 = cv2.imread(template_path_target1, cv2.IMREAD_GRAYSCALE)
template_target2 = cv2.imread(template_path_target2, cv2.IMREAD_GRAYSCALE)

template_path_hotspot1 = 'hotspot_real_smol.png'  # Change this to your hotspot image's path
template_path_hotspot2 = 'hotspot_real_full.png'  # Change this to your hotspot image's path
template_hotspot1 = cv2.imread(template_path_hotspot1, cv2.IMREAD_GRAYSCALE)
template_hotspot2 = cv2.imread(template_path_hotspot2, cv2.IMREAD_GRAYSCALE)


def connectmycopter():

    connection_string = "/dev/serial0"
    baud_rate = 57600
    print("Connecting to drone...")
    #f.write("\n Connecting to drone...")
    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    return vehicle


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
    
    
def get_yaw(vehicle,f):

    print("Attitude:",vehicle.attitude)
    print("YAW:",vehicle.attitude.yaw)
    
    return vehicle.attitude.yaw
    
    
def get_position(vehicle, f):

    print("\nPosition: %s" % vehicle.location.global_relative_frame)
    f.write("\nPosition: %s" % vehicle.location.global_relative_frame)
    return None


def set_home(vehicle, f):

    global flt_alt
    
    home = vehicle.location.global_relative_frame
    home.alt = flt_alt
    
    print("\nHome set at: %s" % vehicle.location.global_relative_frame)
    f.write("\nHome set at: %s" % vehicle.location.global_relative_frame)
    
    return home


def arm_and_takeoff(targetaltitude, vehicle, f):

    while vehicle.is_armable == False:
        print("Waiting for vehicle to become armable")
        f.write("Waiting for vehicle to become armable")
        time.sleep(1)

    vehicle.mode = VehicleMode("STABILIZE")
    print("Current mode: %s" % vehicle.mode.name)
    f.write("\nCurrent mode: %s" % vehicle.mode.name)

    vehicle.armed = True
    while vehicle.armed == False:
        print("Waiting for vehicle to arm")
        f.write("\nWaiting for vehicle to arm")
        time.sleep(1)

    print("VEHICLE ARMED")
    f.write("\n\nVEHICLE ARMED\n")
    print("About to takeoff")
    f.write("\nAbout to takeoff")

    time.sleep(2)

    vehicle.mode = VehicleMode("GUIDED")
    print("\n\nGuided mode\n\n")
    f.write("\n\nGUIDED MODE\n\n")
    	
    while vehicle.mode != "GUIDED":
        print("Waiting for vehicle to enter guided mode")
        f.write("\nWaiting for vehicle to enter guided mode")
        time.sleep(0.3)
    
    time.sleep(3)
    vehicle.simple_takeoff(targetaltitude)

    while True:
        alt_now = vehicle.location.global_relative_frame.alt
        print("Altitude:%f" % alt_now)
        f.write("\nAltitude:%f" % alt_now)
        if alt_now >= flt_alt * 0.98:
            break
        time.sleep(0.3)

    time.sleep(2)
    print("Target altitude reached")
    f.write("\nTarget altitude reached")

    return None


def land_copter(vehicle, f):
	
	vehicle.mode = VehicleMode("LAND")
	print("\n\nLand Mode\n\n")
	f.write("\n\nLand Mode\n\n")
	
	while True:
		alt_now = vehicle.location.global_relative_frame.alt
		print("Altitude:%f"%alt_now)
		f.write("\nAltitude:%f"%alt_now)
		if alt_now <= 0.15:
			break
		time.sleep(0.3)
	
	time.sleep(3)

	print("Landed successfully")
	f.write("\nLanded Successfully\n")

	return None


def coordinate_rotation(yaw_angle, x_cart, y_cart,f):

                    
   # Rotation of point wrt to drone heading
   # Origin at 0,0 required points at x_cart,y_cart rotated point at xr,yr
   
   # Delta is current heading of drone (YAW angle W.R.T North), replace with 0 for testing.
   delta_rad = yaw_angle # Offset angle in Radians
   delta_deg = delta_rad*180/math.pi
   
   #delta_deg = 0
   #delta_rad = delta_deg*math.pi/180 # Offset angle in Radians
 
   xc,yc = center_cord[0],center_cord[1]
                    
   xr = int(((x_cart) * math.cos(delta_rad)) - ((y_cart) * math.sin(delta_rad)));
   yr = int(((x_cart) * math.sin(delta_rad)) + ((y_cart) * math.cos(delta_rad)));
   
   print("Rotated Coordinates:","x:",xr,"y:",yr,"\n")
   print("Delta rad:", delta_rad, "\t\tDelta deg:",delta_deg)
   f.write("Rotated Coordinates: "+" x: "+str(xr)+" y: "+str(yr)+"\n\n")
   f.write("Delta rad: " + str(delta_rad) + "\t\tDelta deg:" + str(delta_deg)+'\n')
      

   return (xr,yr)
   
   
def get_coordinates(yaw_angle, f, circle_x, circle_y):

    h = flt_alt
    if h < 0:
        h = 30   
    x_cart = circle_x - center_cord[0]
    y_cart = center_cord[1] - circle_y
    
    print("Current Altitude:",h)
    print("Cartesian Coordinates:","x:",str(x_cart),"y:",str(y_cart))
    
    f.write("Current Altitude :"+str(h)+'\n')
    f.write("Cartesian Coordinates:"+"x: "+str(x_cart)+" y: "+str(y_cart)+'\n')
    
    xr,yr = coordinate_rotation(yaw_angle, x_cart, y_cart,f)
    
    theta_rad = round(math.atan(10.3/109),5)
    theta_deg = round(theta_rad*180/math.pi,5)
    print("Theta rad:", theta_rad, "\t\tTheta deg:",theta_deg)
    f.write("Theta rad: "+ str(theta_rad)+ " \t\tTheta deg: "+str(theta_deg)+'\n')
    
    if x_cart == 0:
        phi_rad = round(math.pi/2,5)
    else:
       phi_rad = round(math.atan(yr/xr),5)
    
    phi_deg = round(phi_rad*180/math.pi,5)
    print("Phi rad:", phi_rad, "\t\tPhi deg:",phi_deg,"\n")
    f.write("Phi rad "+ str(phi_rad)+ "\t\tPhi deg: "+str(phi_deg)+"\n\n")
    
    Rp = round(math.sqrt(math.pow(xr,2)+math.pow(yr,2)),5) # Rp is pixel distance
    print("Rp (Pixel distance):",Rp)
    f.write("Rp (Pixel distance):"+str(Rp)+'\n')
    
    Rd = round(h*math.tan(theta_rad),5) # Rd is real world distance for 50 pixels
    print("Rd (Real world distance for 50 pixels):",Rd)
    f.write("Rd (Real world distance for 50 pixels): "+str(Rd)+'\n')
    
    d = round((Rp*Rd)/50.0,5) # Real world distance in meters per pixel at height h (flight altitude).
    print("D (Real world Distance to point):",d)
    f.write("D (Real world Distance to point): "+str(d)+'\n')
    
    xd = round(abs(d*math.cos(phi_rad)),5) # Real world distance along x axis (EAST) in meters.
    yd = round(abs(d*math.sin(phi_rad)),5) # Real world distance along y axis (NORTH) in meters.
    
    if xr < 0:
    	xd = xd*-1
    if yr < 0:
    	yd = yd*-1
    print("\nX (Distance along East):",xd,"|| Y (Distance along North):",yd,"\n\n\n")
    f.write("\nX (Distance along East): "+str(xd)+" || Y (Distance along North): "+str(yd)+"\n\n\n")
    
    return (xd,yd)
    
    
def detect_circles(image):

    # Convert the image to grayscale
    # Gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # Converting to greyscale caused hotspot to blend into background
    
    # Apply bilateral filtering to reduce noise and improve circle detection
    #filtered = cv2.bilateralFilter(image, 9, 75, 75)
    #cv2.imshow('Blurred', filtered)
    edges = cv2.Canny(image, threshold1=70, threshold2=155)
    #cv2.imshow('EDGE', edges)
    #cv2.waitKey(0)
    
    # Detect circles using Hough Circle Transform
    circles = cv2.HoughCircles(
        edges, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
        param1=50, param2=30, minRadius=10, maxRadius=120
    )
    '''
    if circles is not None:
        circles = np.uint16(np.around(circles))
            
        for circle in circles[0, :]:
            center = (circle[0], circle[1])
            radius = circle[2]
            
            # Draw the circle outline
            cv2.circle(image, center, radius, (0, 255, 0), 4)   
    
    cv2.imshow("all_circles",image)
    '''
    return circles
    
    
def crop_circles(image, circles):

    cropped_images = []
    
    for circle in circles[0]:
        x, y, r = circle
        ymr = int(y)-int(r)
        ypr = int(y)+int(r)
        xmr = int(x)-int(r)
        xpr = int(x)+int(r)
        
        cropped = image[ymr: ypr, xmr: xpr]
        
        #print(cropped.shape)
        if cropped.shape[0] >= 10 and cropped.shape[1] >=10:
            cropped_images.append(cropped)
    
    return cropped_images


def template_matching(template, image):

    if len(image.shape) == 3:
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        image_gray = image
    
    if template.shape != image_gray.shape:
        template = cv2.resize(template, (image_gray.shape[1], image_gray.shape[0]))
        #cv2.imshow("template",template)
    
    result = cv2.matchTemplate(image_gray, template, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    
    return max_val
    
    
def detect(yaw_angle, f,i,camera):
    
    camera.resolution = (width, height)
    camera.iso = 100
    camera.exposure_mode = 'auto'
    #camera.exposMoonpieure_compensation = -3
    camera.vflip = True
    
    shot_no = str("30m_")+str(i)
    print("Shot NO:",shot_no)
    f.write("\nShot NO:"+shot_no+'\n')
    
    rawCapture = PiRGBArray(camera, size=(width, height))
    
    # allow the camera to warmup
    time.sleep(2)

    camera.capture(rawCapture, format="bgr")
    frame = rawCapture.array
    
    cv2.imwrite("2new_temp"+shot_no+".jpg",frame)
    #cv2.imshow("drone_shot"+shot_no+".jpg",frame)
    poi = list()
    
    try:
        
        # Detect circles in the frame
        detected_circles = detect_circles(frame.copy())
        if detected_circles is None:
            print("\nNo Circles Found\n")
            f.write("\nNo Circles Found\n\n")
            
            poi.append([0,0,"No_circles_found"])      
            return poi
        
        # Crop out detected circles
        cropped_images = crop_circles(frame.copy(), detected_circles)
     
        #perform template matching for each matched circle (2 templates for target and 1 for hotspot)
        for i, cropped in enumerate(cropped_images):
        
            match_target1 = template_matching(template_target1, cropped)
            match_target2 = template_matching(template_target2, cropped)
        
            match_hotspot1 = template_matching(template_hotspot1, cropped)
            match_hotspot2 = template_matching(template_hotspot2, cropped)
        
            if max(match_target1,match_target2,match_hotspot1,match_hotspot2) >= 0.2:
                
                if max(match_target1,match_target2) > max(match_hotspot1,match_hotspot2):
                    target_type = "Target"
                    text_color = (0, 255, 0)  # Green color
                else:
                    target_type = "Hotspot"
                    text_color = (0, 0, 255)  # Red color
            else:
                target_type = "Unknown"
                text_color = (148, 7, 173) # Purple color
                
            print(i,". Target_smol corr: ",match_target1)
            print(i,". Target_full corr: ",match_target2)
            
            print(i,". Hotspot_smol corr: ",match_hotspot1)
            print(i,". Hotspot_full corr: ",match_hotspot2)
            
            f.write(str(i)+". Target_smol corr: "+str(match_target1)+'\n')
            f.write(str(i)+". Target_full corr: "+str(match_target2)+'\n')
            
            f.write(str(i)+". Hotspot_smol corr: "+str(match_hotspot1)+'\n')
            f.write(str(i)+". Hotspot_full corr: "+str(match_hotspot2)+'\n')
            
            # Calculate the position for the target type text
            circle_center = detected_circles[0][i][:2]
            circle_radius = detected_circles[0][i][2]
            circle_center = np.int64(circle_center)
            circle_radius = np.int64(circle_radius)
                
            #print(circle_center," ",circle_radius)
            
            text_position = (circle_center[0] - 40, circle_center[1] + circle_radius + 10)
            
            print("\nTarget Type:",target_type,i)
            print("Pixel Coordinates","X:",circle_center[0],"Y:",circle_center[1])
            f.write("\nTarget Type:"+str(target_type)+str(i)+'\n')
            f.write("Pixel Coordinates"+" X: "+str(circle_center[0])+" Y: "+str(circle_center[1])+'\n')
        
            xd,yd = get_coordinates(yaw_angle, f, circle_center[0], circle_center[1])
                
            # Append all detections to an array
            poi.append([xd,yd,target_type])
                
            line_thickness = 2
            
            cv2.line(frame, (0,center_cord[1]), (width, center_cord[1]), (0, 255, 0), thickness=line_thickness)
            cv2.line(frame, (center_cord[0], 0), (center_cord[0], height), (0, 255, 0), thickness=line_thickness)

            # Add text indicating target type
            cv2.putText(frame, target_type+str(i), text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2)
            # Draw circles 
            cv2.circle(frame, (circle_center[0],circle_center[1]), circle_radius, (0, 255, 0), 4)
                
        # Display the frame with circles   
        #cv2.imshow('Frame with Circles', frame)
        cv2.imwrite("2new_temp_drawn"+shot_no+".jpg", frame)
        #cv2.waitKey(0)        
        cv2.destroyAllWindows()
    
    except Exception as e:
        print(e)
        pass
    
    finally:
        cv2.destroyAllWindows()  # Close any OpenCV windows

    return poi
    
     
if __name__== '__main__':
    
    #vehicle = connectmycopter()
    poi = list()

    f = open("log_target_hotspot_templates_new2.txt", 'w')

    #basic_data(vehicle, f)

    #home_wp = set_home(vehicle, f)
    #time.sleep(2)

    #arm_and_takeoff(flt_alt, vehicle, f)
    #time.sleep(2)
    
    yaw_angle = 0
    flt_alt = 15
    i = 0
    camera = PiCamera()
    
    for i in range(0,30):
        poi = detect(yaw_angle, f,i,camera)
        print("sleeping")
        f.write("\n\nsleeping\n\n")
        time.sleep(2)
        
    #land_copter(vehicle, f)
    #time.sleep(3)

    print("\n--------Mission Successfull--------\n")
    f.write("\n--------Mission Successfull--------")

    #vehicle.armed = False
    #vehicle.close()

