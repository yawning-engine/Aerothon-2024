import cv2
from time import sleep

# Create a VideoCapture object to access the camera
cap = cv2.VideoCapture(0)  # 0 corresponds to the default camera (Raspberry Pi camera module)

try:
    # Allow time for the camera to adjust to lighting conditions
    sleep(2)

    # Capture a single frame
    ret, frame = cap.read()

    # Save the captured frame as an image
    cv2.imshow("capture", frame)
    cv2.waitKey(0)
    # cv2.imwrite('/path/to/image.jpg', frame)

finally:
    # Release the camera
    cap.release()
    cv2.destroyAllWindows()
