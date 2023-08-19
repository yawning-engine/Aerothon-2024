# Function to capture an image from the camera
def capture_image():
    with picamera.PiCamera() as camera:
        camera.resolution = (320, 240)  # Reduce resolution for optimization
        with picamera.array.PiRGBArray(camera) as output:
            camera.capture(output, format="bgr")
            return output.array

# Function to detect the archery circle-like target and hotspots using Hough Circle Detection
def detect_circles(image):
    # Convert image to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise and enhance circle detection
    blurred_image = cv2.GaussianBlur(gray_image, (9, 9), 2)

    # Detect circles using Hough Circle Detection
    circles = cv2.HoughCircles(blurred_image, cv2.HOUGH_GRADIENT, dp=1, minDist=20,
                               param1=CIRCLE_DETECTION_THRESHOLD, param2=30, minRadius=10, maxRadius=70)

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        return circles

    return None

# Function to detect the archery circle-like target and hotspots using Template Matching
def detect_target_and_hotspots(image, circles):
    # Load the template images for the actual target and hotspots
    template_target = cv2.imread("target_template.png", cv2.IMREAD_GRAYSCALE)
    template_hotspot1 = cv2.imread("hotspot1_template.png", cv2.IMREAD_GRAYSCALE)

    # Convert image to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Loop through the detected circles and classify them
    for circle in circles:
        x, y, radius = circle[0], circle[1], circle[2]

        # Extract the region of interest (ROI) around the circle
        roi = gray_image[y - radius:y + radius, x - radius:x + radius]

        # Perform template matching for the ROI
        res_target = cv2.matchTemplate(roi, template_target, cv2.TM_CCOEFF_NORMED)
        res_hotspot1 = cv2.matchTemplate(roi, template_hotspot1, cv2.TM_CCOEFF_NORMED)

        # Define a threshold for considering a match
        match_threshold = 0.7  # You can adjust this threshold based on your use case

        # Check if the best match distance is above the threshold for the target
        if np.max(res_target) > match_threshold:
            return "Archery target", x, y, radius

        # Check if the best match distance is above the threshold for hotspot1
        if np.max(res_hotspot1) > match_threshold:
            return "Hotspot 1", x, y, radius

    return None

# Main code
if __name__ == "__main__":
    while True:
        # Capture image from the camera
        image = capture_image()

        # Detect circles in the image using Hough Circle Detection
        detected_circles = detect_circles(image)

        if detected_circles is not None:
            # Process the image to classify the detected circles as target or hotspot
            result = detect_target_and_hotspots(image, detected_circles)

            if result is not None:
                target_type, target_x, target_y, target_radius = result
                print(f"{target_type} detected at ({target_x}, {target_y}), radius: {target_radius}")

        # Show the image with the detected circles (optional, for visualization)
        if detected_circles is not None:
            for circle in detected_circles:
                x, y, radius = circle[0], circle[1], circle[2]
                cv2.circle(image, (x, y), radius, (0, 255, 0), 2)

        cv2.imshow("Image with Detected Circles", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
