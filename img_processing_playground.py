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


# Initialize webcam capture
cap = cv2.VideoCapture(0)  # 0 indicates the default webcam

try:
    while True:
        ret, frame = cap.read()  # Read a frame from the webcam
        
        if not ret:
            break
        
        # Detect circles in the frame
        frame_with_circles, detected_circles = detect_circles(frame.copy())
        
        # Display the frame with circles
        cv2.imshow('Frame with Circles', frame_with_circles)
        
        # Crop out detected circles
        cropped_images = crop_circles(frame, detected_circles)
        for i, cropped in enumerate(cropped_images):
            cv2.imshow(f'Cropped Circle {i + 1}', cropped)
        
        # Exit the loop on 'q' key press
        if cv2.waitKey(0) & 0xFF == ord('q'):
            break
            
    cv2.destroyAllWindows()
except KeyboardInterrupt:
    pass
finally:
    cap.release()  # Release the webcam
    cv2.destroyAllWindows()  # Close any OpenCV windows

