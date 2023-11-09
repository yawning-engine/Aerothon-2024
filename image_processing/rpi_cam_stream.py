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
	#cv2.namedWindow('Camera Stream', cv2.WINDOW_NORMAL)
	#resize the window according to the screen resolution
	#cv2.resizeWindow('Camera Stream', 800, 600)
	
	image = cv2.circle(image, (320,240), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (420,240), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (520,240), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (220,240), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (120,240), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (620,240), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (20,240), radius=4, color=(0, 0, 255), thickness=-1)
	
	image = cv2.circle(image, (320,140), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (420,140), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (520,140), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (220,140), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (120,140), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (620,140), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (20,140), radius=4, color=(0, 0, 255), thickness=-1)
	
	image = cv2.circle(image, (320,340), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (420,340), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (520,340), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (220,340), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (120,340), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (620,340), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (20,340), radius=4, color=(0, 0, 255), thickness=-1)
	
	image = cv2.circle(image, (320,40), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (420,40), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (520,40), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (220,40), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (120,40), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (620,40), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (20,40), radius=4, color=(0, 0, 255), thickness=-1)
	
	image = cv2.circle(image, (320,440), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (420,440), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (520,440), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (220,440), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (120,440), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (620,440), radius=4, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, (20,440), radius=4, color=(0, 0, 255), thickness=-1)
	
	# show the frame
	cv2.imshow("Camera Stream", image)
	
	key = cv2.waitKey(1) & 0xFF
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
