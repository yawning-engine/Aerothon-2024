import cv2
import numpy as np

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
        edges, cv2.HOUGH_GRADIENT, dp=1, minDist=120,
        param1=50, param2=30, minRadius=15, maxRadius=80
    )
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        
        for circle in circles[0, :]:
            center = (circle[0], circle[1])
            radius = circle[2]
            
            # Draw the circle outline
            cv2.circle(image, center, radius, (0, 255, 0), 4)
    
    return image, circles


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
    
    
# Load the template image of the target and hotspot
template_path_target = 'target.png'  # Change this to your target image's path
template_target = cv2.imread(template_path_target, cv2.IMREAD_GRAYSCALE)
template_path_hotspot = 'hotspot.png'  # Change this to your hotspot image's path
template_hotspot = cv2.imread(template_path_hotspot, cv2.IMREAD_GRAYSCALE)

# Initialize webcam capture
cap = cv2.VideoCapture(0)  # 0 indicates the default webcam

try:
    while True:
        ret, frame = cap.read()  # Read a frame from the webcam
        
        if not ret:
            break
        
        # Detect circles in the frame
        frame_with_circles, detected_circles = detect_circles(frame.copy())
        
        # Crop out detected circles
        cropped_images = crop_circles(frame, detected_circles)
        
        #perform template matching for each matched circle
        for i, cropped in enumerate(cropped_images):
            match_target = template_matching(template_target, cropped)
            match_hotspot = template_matching(template_hotspot, cropped)
            print(match_target)
            print(match_hotspot)
            # Determine target type based on template matching
            if match_target > match_hotspot:
                target_type = "Target"
                text_color = (0, 255, 0)  # Green color
            else:
                target_type = "Hotspot"
                text_color = (0, 0, 255)  # Red color
                
            # Calculate the position for the target type text
            circle_center = detected_circles[0][i][:2]
            circle_radius = detected_circles[0][i][2]
            text_position = (circle_center[0] - 40, circle_center[1] + circle_radius + 10)
            
            # Add text indicating target type
            cv2.putText(frame_with_circles, target_type, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2)
            '''
            # Save the cropped image with target type in the filename
            filename = f'Cropped_Circle_{i + 1}_{target_type}.png'
            cv2.imwrite(filename, cropped)
            '''
            cv2.imshow(f'Cropped Circle {i + 1}', cropped)
        
        # Display the frame with circles
        cv2.imshow('Frame with Circles', frame_with_circles)
        #cv2.imshow('Target', template)
       
        # Exit the loop on 'q' key press
        if cv2.waitKey(0) & 0xFF == ord('q'):
            break
            
    cv2.destroyAllWindows()
except KeyboardInterrupt:
    pass
finally:
    cap.release()  # Release the webcam
    cv2.destroyAllWindows()  # Close any OpenCV windows

