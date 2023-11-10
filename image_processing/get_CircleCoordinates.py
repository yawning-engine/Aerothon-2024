import cv2
import numpy as np
import time
import math
import matplotlib.pyplot as plt

flt_alt = 15
center_cord = [320,240]

def coordinate_rotation(x_cart,y_cart):

                    
   #Rotation of point wrt to drone heading
   #Origin at 0,0 required points at x_cart,y_cart rotated point at xr,yr
   
   delta_deg = 0 # Offset angle in Radians
   delta_rad = delta_deg*math.pi/180 # Offset angle in Radians
 
   xc,yc = center_cord[0],center_cord[1]
                    
   xr = int(((x_cart) * math.cos(delta_rad)) - ((y_cart) * math.sin(delta_rad)));
   yr = int(((x_cart) * math.sin(delta_rad)) + ((y_cart) * math.cos(delta_rad)));
   
   print("Rotated Coordinates:","x:",xr,"y:",yr,"\n")
   print("Delta rad:", delta_rad, "\t\tDelta deg:",delta_deg)
      

   return (xr,yr)
   
   
def get_coordinates(circle_x,circle_y):

    h = flt_alt
    x_cart = circle_x - center_cord[0]
    y_cart = center_cord[1] - circle_y
    
    print("Cartesian Coordinates:","x:",x_cart,"y:",y_cart)
    
    xr,yr = coordinate_rotation(x_cart,y_cart)
    
    theta_rad = round(math.atan(10.3/109),5)
    theta_deg = round(theta_rad*180/math.pi,5)
    print("Theta rad:", theta_rad, "\t\tTheta deg:",theta_deg)
    
    if x_cart == 0:
        phi_rad = round(math.pi/2,5)
    else:
       phi_rad = round(math.atan(yr/xr),5)
    
    phi_deg = round(phi_rad*180/math.pi,5)
    print("Phi rad:", phi_rad, "\t\tPhi deg:",phi_deg,"\n")
    
    Rp = round(math.sqrt(math.pow(xr,2)+math.pow(yr,2)),5) # Rp is pixel distance
    print("Rp (Pixel distance):",Rp)
    
    Rd = round(h*math.tan(theta_rad),5) # Rd is real world distance for 50 pixels
    print("Rd (Real world distance for 50 pixels):",Rd)
    
    d = round((Rp*Rd)/50.0,5) # Real world distance in meters per pixel at height h (flight altitude).
    print("D (Real world Distance to point):",d)
    
    xd = round(abs(d*math.cos(phi_rad)),5) # Real world distance along x axis (EAST) in meters.
    yd = round(abs(d*math.sin(phi_rad)),5) # Real world distance along y axis (NORTH) in meters.
    
    if xr < 0:
    	xd = xd*-1
    if yr < 0:
    	yd = yd*-1
    print("\nX (Distance along East):",xd,"|| Y (Distance along North):",yd,"\n\n\n")
    
    return (xd,yd)
    
    
def detect_circles(image):

    # Apply bilateral filtering to reduce noise and improve circle detection
    filtered = cv2.bilateralFilter(image, 9, 75, 75)
    #cv2.imshow('Blurred', filtered)
    edges = cv2.Canny(filtered, threshold1=70, threshold2=155)
    #cv2.imshow('EDGE', edges)
    #cv2.waitKey(0)
    
    # Detect circles using Hough Circle Transform
    circles = cv2.HoughCircles(
        edges, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
        param1=50, param2=30, minRadius=10, maxRadius=100
    )
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
            
        for circle in circles[0, :]:
            center = (circle[0], circle[1])
            radius = circle[2]
            
            # Draw the circle outline
            cv2.circle(image, center, radius, (0, 255, 0), 4)   
    
    #cv2.imshow("all_circles",image)
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
    
 
# Load the template image of the target and hotspot
template_path_target1 = 'target_real.png'  # Change this to your target image's path
template_path_target2 = 'target_real_full.png'  # Change this to your target image's path

template_target1 = cv2.imread(template_path_target1, cv2.IMREAD_GRAYSCALE)
template_target2 = cv2.imread(template_path_target2, cv2.IMREAD_GRAYSCALE)

template_path_hotspot = 'hotspot_real.png'  # Change this to your hotspot image's path
template_hotspot = cv2.imread(template_path_hotspot, cv2.IMREAD_GRAYSCALE)

img = cv2.imread('tar_hot_1.png', 1)
height = int(img.shape[0]*0.5)
width = int(img.shape[1]*0.5)
center_cord[1] = int(height/2)
center_cord[0] = int(width/2)

frame = cv2.resize(img, (width, height), interpolation = cv2.INTER_LINEAR)

try:
    # Detect circles in the frame
    detected_circles = detect_circles(frame.copy())
        
    # Crop out detected circles
    cropped_images = crop_circles(frame.copy(), detected_circles)
     
    #perform template matching for each matched circle (2 templates for target and 1 for hotspot)
    for i, cropped in enumerate(cropped_images):
        
        match_target1 = template_matching(template_target1, cropped)
        match_target2 = template_matching(template_target2, cropped)
        
        match_hotspot = template_matching(template_hotspot, cropped)
        
        if max(match_target1,match_target2,match_hotspot) >= 0.5:
            print(i,". Target_smol corr: ",match_target1)
            print(i,". Target_full corr: ",match_target2)
            
            print(i,". Hotspot corr: ",match_hotspot)
            if max(match_target1,match_target2) > match_hotspot:
                target_type = "Target"
                text_color = (0, 255, 0)  # Green color
            else:
                target_type = "Hotspot"
                text_color = (0, 0, 255)  # Red color
                
            # Calculate the position for the target type text
            circle_center = detected_circles[0][i][:2]
            circle_radius = detected_circles[0][i][2]
            #print(circle_center," ",circle_radius)
            
            text_position = (circle_center[0] - 40, circle_center[1] + circle_radius + 10)
            
            print("\nTarget Type:",target_type)
            print("Pixel Coordinates","X:",circle_center[0],"Y:",circle_center[1])
        
            xd,yd = get_coordinates(circle_center[0],circle_center[1])
                
            line_thickness = 2
            
            cv2.line(frame, (0,center_cord[1]), (width, center_cord[1]), (0, 255, 0), thickness=line_thickness)
            cv2.line(frame, (center_cord[0], 0), (center_cord[0], height), (0, 255, 0), thickness=line_thickness)

            # Add text indicating target type
            cv2.putText(frame, target_type, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2)
            # Draw circles 
            cv2.circle(frame, (circle_center[0],circle_center[1]), circle_radius, (0, 255, 0), 4)

            
        # Display the frame with circles
        cv2.imshow('Frame with Circles', frame)
    
    cv2.waitKey(0)        
    cv2.destroyAllWindows()
    
except KeyboardInterrupt:
    pass
    
finally:
    cv2.destroyAllWindows()  # Close any OpenCV windows

