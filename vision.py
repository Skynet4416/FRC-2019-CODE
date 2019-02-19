import cv2
from smbus2 import *
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import math

# exit key
EXIT_KEY = ord("q")
# filter
LOW = [50, 180, 40]
HIGH = [70, 255, 255]
# recognition
MIN_AREA = 30
ERROR = 0.3
# morphology
KER_SIZE = 3
ITERATIONS = 5
# mark distance calculation
T_HEIGHT = 0.148
RESOLUTION = (640, 480)
V_VIEW_ANGLE = math.radians(10.6) # vertical
# final calculation

def light_on():
	with SMBusWrapper(1) as bus:
		msg = i2c_msg.write(0x70, [0x00, 0xFF])	# turn on i2c msg
		bus.i2c_rdwr(msg)

def light_off():
	with SMBusWrapper(1) as bus:
		msg = i2c_msg.write(0x70, [0x00, 0x00])	# turn off i2c msg
		bus.i2c_rdwr(msg)

def pt_distance(a, b):
	return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

# light_on()
camera = PiCamera()
camera.resolution = RESOLUTION
camera.framerate = 50
camera.iso = 270
camera.shutter_speed = 750
rawCapture = PiRGBArray(camera, size=RESOLUTION)
for frame in camera.capture_continuous(rawCapture, format="bgr",
					use_video_port=True):
	image = frame.array		# convert to cv2 handleable format
	#cv2.imshow("Raw", image)
	pts = []
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)	# frame to hsv
	mask = cv2.inRange(hsv, np.array(LOW), np.array(HIGH))
	ker = cv2.getStructuringElement(cv2.MORPH_OPEN, (KER_SIZE, KER_SIZE))
	mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, ker, iterations = ITERATIONS)
	filtered = cv2.bitwise_and(image, image, mask=mask)
	#cv2.imshow("Filtered", filtered)	# display  filtered frame
	_, contours, __ = cv2.findContours(mask, cv2.RETR_TREE,
					cv2.CHAIN_APPROX_SIMPLE)
	for contour in contours:
		cnt_area = cv2.contourArea(contour)
		if cnt_area < MIN_AREA:
			continue
		rect = cv2.minAreaRect(contour)
		box = np.int0(cv2.boxPoints(rect))
		distances = [pt_distance(box[0], box[1]),
			pt_distance(box[0], box[2]), pt_distance(box[0], box[3])]
		distances.sort()
		rect_area = distances[0] * distances[1]
		if rect_area / cnt_area > 1 + ERROR:
			continue
		cv2.drawContours(image, [box], 0, (0,0,255), 2)
		cnt_x, cnt_y, cnt_w, cnt_h = cv2.boundingRect(contour)
		cnt_dist = T_HEIGHT * RESOLUTION[1] / (2 * math.tan(V_VIEW_ANGLE) * cnt_h)
		cv2.putText(image, str(round(cnt_dist, 2)), (cnt_x, cnt_y),
			cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
		pts.append((cnt_x, cnt_dist))
	if len(pts) == 2:
		if pts[0][0] > pts[1][0]:
			y = pts[0][1]
			x = pts[1][1]
		else:
			x = pts[0][1]
			y = pts[1][1]
		dist = math.sqrt((x**2 + y**2 - 0.02)/2)
		cv2.putText(image, "dist=" + str(round(dist, 2)), (0, RESOLUTION[1]),
			cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
	cv2.imshow("contours", image)
	key = cv2.waitKey(1) & 0xFF	# if exit key is pressed
	rawCapture.truncate(0)		# clear stream for next image
	if key == EXIT_KEY:		# exit
		break
# light_off()

