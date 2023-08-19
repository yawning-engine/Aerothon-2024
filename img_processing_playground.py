import cv2
import numpy as np

def detect_circles(image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply bilateral filtering to reduce noise and improve circle detection
    filtered = cv2.bilateralFilter(gray, 9, 75, 75)
    
    cv2.imshow('Blurred', filtered)
    
    edges = cv2.Canny(filtered, threshold1=50, threshold2=150)
    
    cv2.imshow('EDGE', edges)
    
    # Detect circles using Hough Circle Transform
    circles = cv2.HoughCircles(
        edges, cv2.HOUGH_GRADIENT, dp=1, minDist=90,
        param1=50, param2=30, minRadius=15, maxRadius=60
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
    
    result = cv2.matchTemplate(image_gray, template, cv2.TM_CCOEFF_NORMED)
    _, _, _, max_loc = cv2.minMaxLoc(result)
    return max_loc
    
    
# Load the template image of the target
template_path = 'target.jpeg'  # Change this to your template image's path
template = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)

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
            match_location = template_matching(template, cropped)
            x, y = match_location
            h, w = template.shape
            
            # Draw a rectangle around the matched area
            cv2.rectangle(cropped, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
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

