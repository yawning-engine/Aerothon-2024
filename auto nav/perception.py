# import the necessary packages
from dronekit import connect
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
import numpy as np
import time
import math

# Required global variables
yaw_angle = 0
img_count = 0
wp_count = 0
flt_alt = 30
img_cord_list = list()

# Initialize Picam capture
width = 640
height = 480
center_cord = [int(width/2),int(height/2)]

camera = PiCamera()
camera.resolution = (width, height)
camera.iso = 100
camera.exposure_mode = 'auto'
#camera.exposMoonpieure_compensation = -3
camera.vflip = True


# Load the template image of the target and hotspot
template_path_target1 = 'templates/target_real_full.png'  # Change this to your target image's path
template_path_target2 = 'templates/target_real_smol.png'  # Change this to your target image's path

template_target1 = cv2.imread(template_path_target1, cv2.IMREAD_GRAYSCALE)
template_target2 = cv2.imread(template_path_target2, cv2.IMREAD_GRAYSCALE)

template_path_hotspot1 = 'templates/hotspot_real_full.png'  # Change this to your hotspot image's path
template_path_hotspot2 = 'templates/hotspot_real_smol.png'  # Change this to your hotspot image's path
template_hotspot1 = cv2.imread(template_path_hotspot1, cv2.IMREAD_GRAYSCALE)
template_hotspot2 = cv2.imread(template_path_hotspot2, cv2.IMREAD_GRAYSCALE)
    
    
def take_picture(f):
    
    global img_count
    rawimg = PiRGBArray(camera, size=(width, height))
    
    # allow the camera to warmup
    time.sleep(2)

    camera.capture(rawimg, format="bgr")
    img = rawimg.array
    
    print("Image_NO:", img_count)
    f.write("Image_NO:" + str(img_count) + "\n")
    
    cv2.imwrite("Image_NO" + str(img_count) + ".jpg",img)
    
    img_count = img_count + 1
    
    
def get_yaw(vehicle, f):

    yaw = vehicle.attitude.yaw
    
    print("Current heading:", yaw)
    f.write("Current heading:" + str(yaw) + "\n")
    
    return yaw


def get_alt(vehicle, f):

    h = vehicle.location.global_relative_frame.alt
    
    print("Current altitude:", h)
    f.write("Current altitude:" + str(h) + "\n")
    
    return h


def coordinate_rotation(x_cart, y_cart, f):
                 
   # Rotation of point wrt to drone heading
   # Origin at 0,0 required points at x_cart,y_cart rotated point at xr,yr
   
   # Delta is current heading of drone (YAW angle W.R.T North), replace with 0 for testing.
   delta_rad = yaw_angle # Offset angle in Radians
   delta_deg = delta_rad*180/math.pi
   
   xc,yc = center_cord[0],center_cord[1]
                    
   xr = int(((x_cart) * math.cos(delta_rad)) - ((y_cart) * math.sin(delta_rad)));
   yr = int(((x_cart) * math.sin(delta_rad)) + ((y_cart) * math.cos(delta_rad)));
   
   print("Rotated Coordinates:", "x:", xr, "y:", yr, "\n")
   print("Delta rad:", delta_rad, "\t\tDelta deg:", delta_deg)
   f.write("Rotated Coordinates: "+" x:"+str(xr)+" y:"+str(yr)+"\n")
   f.write("Delta rad:" + str(delta_rad) + "\t\tDelta deg:" + str(delta_deg) + "\n")
      
   return (xr,yr)
   
   
def get_coordinates(flt_alt, yaw_angle, circle_x, circle_y, f):

    h = flt_alt
    
    x_cart = circle_x - center_cord[0]
    y_cart = center_cord[1] - circle_y
    
    print("Current Altitude:", h)
    print("Cartesian Coordinates:", "x:", str(x_cart), "y:", str(y_cart))
    f.write("Current Altitude :" + str(h) + "\n")
    f.write("Cartesian Coordinates: " + "x:" + str(x_cart) + " y:" + str(y_cart) + "\n")
    
    xr,yr = coordinate_rotation(yaw_angle, x_cart, y_cart,f)
    
    theta_rad = round(math.atan(10.3/109),5)
    theta_deg = round(theta_rad*180/math.pi,5)
    
    print("Theta rad:", theta_rad, "\t\tTheta deg:", theta_deg)
    f.write("Theta rad:" + str(theta_rad) + "\t\tTheta deg:" + str(theta_deg) + "\n")
    
    if x_cart == 0:
        phi_rad = round(math.pi/2,5)
    else:
       phi_rad = round(math.atan(yr/xr),5)
    
    phi_deg = round(phi_rad*180/math.pi,5)
    print("Phi rad:", phi_rad, "\t\tPhi deg:", phi_deg, "\n")
    f.write("Phi rad:" + str(phi_rad) + "\t\tPhi deg:" + str(phi_deg) + "\n\n")
    
    Rp = round(math.sqrt(math.pow(xr,2)+math.pow(yr,2)),5) # Rp is pixel distance
    print("Rp (Pixel distance):", Rp)
    f.write("Rp (Pixel distance):" + str(Rp) + "\n")
    
    Rd = round(h*math.tan(theta_rad),5) # Rd is real world distance for 50 pixels
    print("Rd (Real world distance for 50 pixels):",Rd)
    f.write("Rd (Real world distance for 50 pixels):" + str(Rd) + "\n")
    
    d = round((Rp*Rd)/50.0,5) # Real world distance in meters per pixel at height h (flight altitude).
    print("D (Real world Distance to point):", d)
    f.write("D (Real world Distance to point):" + str(d) + "\n")
    
    xd = round(abs(d*math.cos(phi_rad)),5) # Real world distance along x axis (EAST) in meters.
    yd = round(abs(d*math.sin(phi_rad)),5) # Real world distance along y axis (NORTH) in meters.
    
    if xr < 0:
        xd = xd*-1  
    if yr < 0:
        yd = yd*-1
    
    print("\nX (Distance along East):", xd, "|| Y (Distance along North):", yd, "\n\n\n")
    f.write("\nX (Distance along East):" + str(xd) + "|| Y (Distance along North): "+str(yd)+"\n\n\n")
    
    return (xd,yd)
    
    
def detect_circles(image):
 
    # Apply bilateral filtering to reduce noise and improve circle detection
    filtered = cv2.bilateralFilter(image, 9, 75, 75)
    
    # Extract contours for circle detection
    edges = cv2.Canny(image, threshold1=70, threshold2=155)
    
    # Detect circles using Hough Circle Transform
    circles = cv2.HoughCircles(
        edges, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
        param1=50, param2=30, minRadius=10, maxRadius=120
    )
    
    return circles
    
    
def crop_circles(image, circles):

    cropped_images = []
    
    for circle in circles[0]:
        x, y, r = circle
        # x,y,r are uint8 and may cause integer overflow on adding 
        ymr = int(y)-int(r)
        ypr = int(y)+int(r)
        xmr = int(x)-int(r)
        xpr = int(x)+int(r)
        
        cropped = image[ymr: ypr, xmr: xpr]
        
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
        
    result = cv2.matchTemplate(image_gray, template, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    
    return max_val
    
    
# Returns [xd(float), yd(float), type(string)]
# xd: Distance to target along East      
# xd: Distance to target along North      
# type: Taget type => target / hotspot / no_circles       

def detect(vehicle, f):
    
    rawCapture = PiRGBArray(camera, size=(width, height))
    
    # allow the camera to warmup
    time.sleep(2)

    camera.capture(rawCapture, format="bgr")
    frame = rawCapture.array
    
    print("wp_shot NO:", wp_count)
    f.write("wp_shot NO:" + wp_count + "\n")
    
    cv2.imwrite("wp_shot" + wp_count + ".jpg",frame)
    
    wp_count = wp_count + 1
    
    # List to store coordinates of all detected circles in frame
    poi = list()
    
    try:
        # Detect circles in the frame
        detected_circles = detect_circles(frame.copy())
        
        if detected_circles is None:
            print("\nNo Circles Found\n")
            f.write("\nNo Circles Found\n\n")
            
            poi.append([0, 0, "no_circles"])
                  
            return poi
        
        yaw_angle = get_yaw(vehicle, f)
        flt_alt = get_alt(vehicle, f)
        
        # Crop out detected circles
        cropped_images = crop_circles(frame.copy(), detected_circles)
     
        #perform template matching for each matched circle (2 templates for target and 1 for hotspot)
        for i, cropped in enumerate(cropped_images):
        
            match_target1 = template_matching(template_target1, cropped)
            match_target2 = template_matching(template_target2, cropped)
        
            match_hotspot1 = template_matching(template_hotspot1, cropped)
            match_hotspot2 = template_matching(template_hotspot2, cropped)
        
            if max(match_target1,match_target2,match_hotspot1,match_hotspot2) >= 0.3:
                
                if max(match_target1,match_target2) > max(match_hotspot1,match_hotspot2):
                    target_type = "Target"
                    text_color = (0, 255, 0)  # Green color
                else:
                    target_type = "Hotspot"
                    text_color = (0, 0, 255)  # Red color
            else:
                target_type = "Unknown"
                text_color = (148, 7, 173) # Purple color
                
            print(i, ". Target_1 corr:", match_target1)
            print(i, ". Target_2 corr:", match_target2)
            
            print(i, ". Hotspot_1 corr:", match_hotspot1)
            print(i, ". Hotspot_2 corr:", match_hotspot2)
            
            f.write(str(i) + ". Target_1 corr:" + str(match_target1) + "\n")
            f.write(str(i) + ". Target_2 corr:" + str(match_target2) + "\n")
            
            f.write(str(i) + ". Hotspot_1 corr:" + str(match_hotspot1) + "\n")
            f.write(str(i) + ". Hotspot_2 corr:" + str(match_hotspot2) + "\n")
            
            # Calculate the position for the target type text
            circle_center = detected_circles[0][i][:2]
            circle_radius = detected_circles[0][i][2]
            circle_center = np.int64(circle_center)
            circle_radius = np.int64(circle_radius)
                
            text_position = (circle_center[0] - 40, circle_center[1] + circle_radius + 10)
            
            print("\nTarget Type:", target_type, i)
            print("Pixel Coordinates", "X:", circle_center[0], "Y:", circle_center[1])
            f.write("\nTarget Type:" + str(target_type) + str(i) + "\n")
            f.write("Pixel Coordinates: " + "X:" + str(circle_center[0]) + " Y:" + str(circle_center[1]) + "\n")
        
            xd,yd = get_coordinates(flt_alt, yaw_angle, circle_center[0], circle_center[1], f)
                
            # Append all detections to an array
            poi.append([xd,yd,target_type])
                
            line_thickness = 2
            
            cv2.line(frame, (0,center_cord[1]), (width, center_cord[1]), (0, 255, 0), thickness=line_thickness)
            cv2.line(frame, (center_cord[0], 0), (center_cord[0], height), (0, 255, 0), thickness=line_thickness)

            # Add text indicating target type
            cv2.putText(frame, target_type+str(i), text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2)
            # Draw circles 
            cv2.circle(frame, (circle_center[0],circle_center[1]), circle_radius, (0, 255, 0), 4)
                
        cv2.destroyAllWindows()
    
    except KeyboardInterrupt:
        pass

    finally:
        cv2.destroyAllWindows()  # Close any OpenCV windows

    return poi
      
