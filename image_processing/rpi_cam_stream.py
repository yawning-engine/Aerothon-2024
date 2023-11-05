#Accessing the Raspberry Pi Camera with OpenCV and Python
#raspistill -o output.jpg

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
# allow the camera to warmup
time.sleep(0.1)
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array
	
	#cv2.WINDOW_NORMAL makes the output window resizealbe
	cv2.namedWindow('Camera Stream', cv2.WINDOW_NORMAL)
	#resize the window according to the screen resolution
	cv2.resizeWindow('Camera Stream', 800, 600)
	
	# show the frame
	cv2.imshow("Camera Stream", image)
	key = cv2.waitKey(1) & 0xFF
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
