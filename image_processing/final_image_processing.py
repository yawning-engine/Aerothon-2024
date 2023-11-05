import cv2
import numpy as np
import time

flt_alt = 15

# Load the template image of the target and hotspot
template_path_target = 'target.png'  # Change this to your target image's path
template_target = cv2.imread(template_path_target, cv2.IMREAD_GRAYSCALE)
template_path_hotspot = 'hotspot.png'  # Change this to your hotspot image's path
template_hotspot = cv2.imread(template_path_hotspot, cv2.IMREAD_GRAYSCALE)

centers = list()
center_cord = [320,240]


def get_location_metres(vehicle, dNorth, dEast, f):

    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    
    global flt_alt
    
    earth_radius = 6378137.0  # Radius of "spherical" earth
    lat_now = vehicle.location.global_relative_frame.lat
    lon_now = vehicle.location.global_relative_frame.lon
    alt_now = vehicle.location.global_relative_frame.alt
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * lat_now / 180))

    # New position in decimal degrees
    newlat = lat_now + (dLat * 180 / math.pi)
    newlon = lon_now + (dLon * 180 / math.pi)

    if type(vehicle.location.global_relative_frame) is LocationGlobalRelative:
        targetlocation = LocationGlobalRelative(newlat, newlon, flt_alt)
    else:
        raise Exception("Invalid Location object passed")

    print("New location coordinates: %s"%str(targetlocation))
    f.write("\nNew location coordinates: %s"%str(targetlocation))

    return targetlocation
    

def detect_circles(image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply bilateral filtering to reduce noise and improve circle detection
    filtered = cv2.bilateralFilter(gray, 9, 75, 75)
    
    #cv2.imshow('Blurred', filtered)
    
    edges = cv2.Canny(filtered, threshold1=50, threshold2=150)
    
    cv2.imshow('EDGE', edges)
    
    # Detect circles using Hough Circle Transform
    circles = cv2.HoughCircles(
        edges, cv2.HOUGH_GRADIENT, dp=1, minDist=80,
        param1=50, param2=30, minRadius=10, maxRadius=80
    )
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        ''''    
        for circle in circles[0, :]:
            center = (circle[0], circle[1])
            radius = circle[2]
            
            # Draw the circle outline
            cv2.circle(image, center, radius, (0, 255, 0), 4)
    '''
    return circles
    
    
def crop_circles(image, circles):
    cropped_images = []
    
    for circle in circles[0]:
        x, y, r = circle
        cropped = image[y - r: y + r, x - r: x + r]
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
    
    
def detect():
    
    # Initialize webcam capture
    height = 480
    width = 640
    
    cap = cv2.VideoCapture(0)  # 0 indicates the default webcam
    cap.set(4, height)
    cap.set(3, width)
    
    center_frame = []
    try:
        while True:
            ret, frame = cap.read()  # Read a frame from the webcam
            if not ret:
                break
        
            # Detect circles in the frame
            detected_circles = detect_circles(frame)
        
            # Crop out detected circles
            cropped_images = crop_circles(frame, detected_circles)
        
            #perform template matching for each matched circle
            for i, cropped in enumerate(cropped_images):
                match_target = template_matching(template_target, cropped)
                match_hotspot = template_matching(template_hotspot, cropped)
                # Determine target type based on template matching
                if max(match_target,match_hotspot) > 0.5:
                    #print(i,". Target corr: ",match_target)
                    #print(i,". Hotspot corr: ",match_hotspot)
                    if match_target > match_hotspot:
                        target_type = "Target"
                        text_color = (0, 255, 0)  # Green color
                    else:
                        target_type = "Hotspot"
                        text_color = (0, 0, 255)  # Red color
                
                    # Calculate the position for the target type text
                    circle_center = detected_circles[0][i][:2]
                    circle_radius = detected_circles[0][i][2]
                    print("Center:",circle_center,"Radius:",circle_radius)
                    text_position = (circle_center[0] - 40, circle_center[1] + circle_radius + 10)
                    
                    alt = flt_alt
                    px_width = 560/alt
                    x1=-(center_cord[0]-detected_circles[0][i][0])/px_width
                    y1=(center_cord[1]-detected_circles[0][i][1])/px_width
                    print("\nX:",x1,"Y:",y1,"\n")
                    
                    #Rotation of point wrt to drone heading
                    #Origin at x0,y0, required points at x1,y1 rotated point at x2,y2
                    
                    x0,y0 = center_cord[0],center_cord[1]
                    
                    x2 = ((x1 - x0) * cos(a)) - ((y1 - y0) * sin(a)) + x0;
                    y2 = ((x1 - x0) * sin(a)) + ((y1 - y0) * cos(a)) + y0;
                    
                    #target_location = get_location_metres(vehicle, x2, y2, f)
                    
                    center_frame.append(circle_center)
            
                    # Add text indicating target type
                    cv2.putText(frame, target_type, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2)
                    #draw circles 
                    cv2.circle(frame, (circle_center[0],circle_center[1]), circle_radius, (0, 255, 0), 4)
                    '''
                    # Save the cropped image with target type in the filename
                    filename = f'Cropped_Circle_{i + 1}_{target_type}.png'
                    cv2.imwrite(filename, cropped)
                    '''
                    #cv2.imshow(f'Cropped Circle {i + 1}', cropped)
        
            # Display the frame with circles
            cv2.imshow('Frame with Circles', frame)
            #cv2.imshow('Target', template)
            #print("\n\n\nExecution time:",time.time()-start_time)
            # Exit the loop on 'q' key press
            if cv2.waitKey(0) & 0xFF == ord('q'):
                break
            
        cv2.destroyAllWindows()
    except error as e:
        print(e)
        pass
    finally:
        cap.release()  # Release the webcam
        cv2.destroyAllWindows()  # Close any OpenCV windows
        return center_frame


centers = detect()
#print(centers)
    

