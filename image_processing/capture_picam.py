from picamera import PiCamera
from time import sleep

camera = PiCamera()

try:
    # Set a fixed ISO value (e.g., 100)
    #camera.iso = 0

    # Allow time for the camera to adjust to lighting conditions
    sleep(2)

    # Set the exposure mode (try different modes)
    # camera.exposure_mode = 'auto'
    #camera.exposure_compensation = -3

    # Set a fixed shutter speed (in microseconds, e.g., 1000 for 1ms)
    # camera.shutter_speed = 0

    # Capture an image
    camera.capture('image_testing.jpg')

finally:
    camera.close()
