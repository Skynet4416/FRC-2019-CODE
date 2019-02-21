import cv2
from smbus2 import *
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import math
import mp_planner
import network_table_profile_api as ntapi
from time import time
from threading import Thread, Condition


g_frame = None
g_frame_cond = Condition()
g_frame_pred = False

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
V_VIEW_ANGLE = math.radians(7.89) # vertical
TARGET_DIST = 0.20
V_MARKER_RATIO = 2.75 # Ratio between the marker height and marker width.
# final calculation
QUEUE_SIZE = 20	
g_running = True

def acquire_frames():
	global g_frame_pred
	global g_frame
	
	raw_capture = PiRGBArray(camera, size=RESOLUTION)

	for frame in camera.capture_continuous(raw_capture, format="bgr",
					use_video_port=True):
		g_frame = frame
	
		with g_frame_cond:
			g_frame_pred = True
			g_frame_cond.notify()
			
		raw_capture.truncate(0)  # clear stream for next image
		
		if not g_running:
			break
			
	

def toggle_light(state: bool):
	with SMBusWrapper(1) as bus:
		msg = i2c_msg.write(0x70, [0x00, 0xFF if state else 0x00])  # ternary for if statement
		bus.i2c_rdwr(msg)

def pt_distance(a, b):
	return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def target_details(x, y, dist):
	is_reversed = False
	if (x > y):
		x, y = y, x
		is_reversed = True
	beta = math.acos((x**2+y**2-TARGET_DIST)/(2*x*y))
	gamma = math.acos((-x**2+y**2-TARGET_DIST)/(2*TARGET_DIST*x))
	
	alpha = math.atan((y*math.sin(gamma-beta)-x*math.sin(gamma))/
			(x*math.cos(gamma)-y*math.cos(gamma-beta)+TARGET_DIST))
	
	delta = math.acos((-TARGET_DIST**2+4*x**2+4*dist**2)/(8*x*dist))
	epsilon = math.pi + delta - gamma - alpha
	target_x = dist*math.cos(epsilon)
	target_y = dist*math.sin(epsilon)
	
	if (is_reversed):
		# axis is negative
		return ((math.pi / 2) + alpha, -target_x, target_y)
	
	return ((math.pi / 2) - alpha, target_x, target_y)

# toggle_light(True)
camera = PiCamera()
camera.resolution = RESOLUTION
camera.framerate = 90
camera.iso = 270
camera.rotation = 270
camera.shutter_speed = 750

def get_contours(image):
	"""
	@param image: A regular bgr image
	"""
	
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)	# frame to hsv
	mask = cv2.inRange(hsv, np.array(LOW), np.array(HIGH))
	ker = cv2.getStructuringElement(cv2.MORPH_OPEN, (KER_SIZE, KER_SIZE))
	# mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, ker, iterations = ITERATIONS)
	filtered = cv2.bitwise_and(image, image, mask=mask)
	#cv2.imshow("Filtered", filtered)	# display  filtered frame
	_, contours, __ = cv2.findContours(mask, cv2.RETR_TREE,
					cv2.CHAIN_APPROX_SIMPLE)

	return contours


def main():
	last_sec = time()
	frames_since = 0
	curr_fps = 0
	
	x_queue = []
	y_queue = []
	
	while True:
		with g_frame_cond:
			if not g_frame_pred:
				g_frame_cond.wait()

		image = g_frame.array		# convert to cv2 handleable format
		#cv2.imshow("Raw", image)
		pts = []
		contours = get_contours(image)
	
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

			# If the ratio isn't between 2.45 and 3.05, continue
			if abs((distances[1]/distances[0])-V_MARKER_RATIO) > ERROR * 3:
				continue
			
			cnt_x, cnt_y, cnt_w, cnt_h = cv2.boundingRect(contour)
			cv2.drawContours(image, [box], 0, (0,0,255), 2)

			#print("Contour ratio: {}".format(distances[1]/distances[0]))
			cnt_dist = T_HEIGHT * RESOLUTION[1] / (2 * math.tan(V_VIEW_ANGLE) * cnt_h)
			cv2.putText(image, str(round(cnt_dist, 2)), (cnt_x, cnt_y),
				cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
			pts.append((cnt_x, cnt_dist))
		
		if len(pts) == 2:
			frames_since += 1
			if int(time()) != last_sec:
				curr_fps = frames_since
				frames_since = 0
				last_sec = int(time())
			
			if pts[0][0] > pts[1][0]:
				y = pts[0][1]
				x = pts[1][1]
			else:
				x = pts[0][1]
				y = pts[1][1]
			x_queue.append(x)
			y_queue.append(y)
			
			if len(x_queue) == QUEUE_SIZE + 1:
				x_queue.pop(0)
				y_queue.pop(0)
				
				x_avg = sum(x_queue)/QUEUE_SIZE
				y_avg = sum(y_queue)/QUEUE_SIZE
				dist = math.sqrt((x_avg**2 + y_avg**2 - 0.02)/2)
				angle, target_x, target_y = target_details(x_avg, y_avg,
														   dist)
				
				if ntapi.get_bool(ntapi.VISION_START_KEY, False):
					
					left_prof, right_prof = mp_planner.motion_profiler(target_x,
																target_y, angle)
					ntapi.send_motion_profiles(left_prof, right_prof)
				
				info = "dist={:.2f} angle={:.2f} fps={}" \
				.format(dist, math.degrees(angle), curr_fps)
				
				cv2.putText(image, info, (0, RESOLUTION[1] - 15),
					cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
		else:
			x_queue = []
			y_queue = []

		cv2.imshow("contours", image)
		key = cv2.waitKey(1) & 0xFF  # if exit key is pressed

		if key == EXIT_KEY:  # exit
			raise KeyboardInterrupt
	# toggle_light(False)
		
	

if __name__ == "__main__":
	thread = Thread(target=acquire_frames)
	thread.start()
	
	# ntapi.init_and_wait()
	while True:
		try:
			main()
		except KeyboardInterrupt:
			break
		except Exception as e: # o byle please forgive me
			print("died with {}\nrestarting..." .format(e))
	
	print("done")
	g_running = False
