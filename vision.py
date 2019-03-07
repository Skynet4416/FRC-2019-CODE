import cv2
from smbus2 import *
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import math
# import mp_planner
from mp_planner import motion_profiler
import network_table_profile_api as ntapi
from time import time
from threading import Thread, Condition

g_frame = None
g_frame_cond = Condition()
g_frame_pred = False
g_running = True

INFO_TEMPLATE = "dist={:.2f} angle={:.2f} fps={}"
# Most important constant
MAIN = "__main__"
# Exit key
EXIT_KEY = ord("q")
# Filter
LOW = [50, 180, 40]
HIGH = [70, 255, 255]
# Recognition
MIN_AREA = 30
ERROR = 0.3
# Morphology
KER_SIZE = 3
ITERATIONS = 5
# Mark distance calculation
T_HEIGHT = 0.148
RESOLUTION = (640, 480)
V_VIEW_ANGLE = math.radians(7.89)  # Vertical
TARGET_DIST = 0.20  # Distance between the two light thingies
V_MARKER_RATIO = 2.75  # Ratio between the marker height and marker width.
TEXT_COLOUR = (255, 255, 255)  # Colour of surrounding text
BOX_COLOUR = (0, 0, 255)  # Colour of box
# final calculation
QUEUE_SIZE = 20


def acquire_frames():
    """
    Acquires frames
    """
    global g_frame_pred
    global g_frame

    raw_capture = PiRGBArray(camera, size=RESOLUTION)

    for frame in camera.capture_continuous(raw_capture, format="bgr",
                                           use_video_port=True):
        g_frame = frame

        with g_frame_cond:
            g_frame_pred = True
            g_frame_cond.notify()

        raw_capture.truncate(0)  # clears stream for next image

        if not g_running:
            break


def toggle_light(state: bool):
    """
    Toggles the lights
    """
    with SMBusWrapper(1) as bus:
        # ternary for if statement
        msg = i2c_msg.write(0x70, [0x00, 0xFF if state else 0x00])
        bus.i2c_rdwr(msg)


def pt_distance(a, b):
    """
    Returns the distance between point a and point b
    """
    return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def target_details(x, y, dist):
    """
    Does some black magic Kalisch calculated somehow
    Returns something that works somehow
    """
    is_reversed = False
    if x > y:
        x, y = y, x
        is_reversed = True
    beta = math.acos((x ** 2 + y ** 2 - TARGET_DIST) / (2 * x * y))
    gamma = math.acos((-x ** 2 + y ** 2 - TARGET_DIST) / (2 * TARGET_DIST * x))

    temp = gamma - beta
    alpha = math.atan((y * math.sin(temp) - x * math.sin(gamma)) /
                      (x * math.cos(gamma) - y * math.cos(temp) + TARGET_DIST))

    delta = math.acos((-TARGET_DIST ** 2 + 4 * x ** 2 + 4 * dist ** 2) /
                      (8 * x * dist))
    epsilon = math.pi + delta - gamma - alpha
    target_x = dist * math.cos(epsilon)
    target_y = dist * math.sin(epsilon)

    if is_reversed:
        # axis is negative
        return (math.pi / 2) + alpha, -target_x, target_y

    return (math.pi / 2) - alpha, target_x, target_y


# toggle_light(True)
camera = PiCamera()
camera.resolution = RESOLUTION
camera.framerate = 90
camera.iso = 270
camera.rotation = 270
camera.shutter_speed = 750


def get_contours(image):
    """
    :param image: A regular bgr image
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)    # frame to hsv
    mask = cv2.inRange(hsv, np.array(LOW), np.array(HIGH))
    ker = cv2.getStructuringElement(cv2.MORPH_OPEN, (KER_SIZE, KER_SIZE))
    # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
    #                         ker, iterations=ITERATIONS)
    filtered = cv2.bitwise_and(image, image, mask=mask)
    # cv2.imshow("Filtered", filtered)  # display  filtered frame
    _, contours, __ = cv2.findContours(mask, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)

    return contours


def main():
    """
    Does whatever main should be doing
    """
    global g_frame_pred
    last_sec = time()
    frames_since = 0
    curr_fps = 0

    x_queue = []
    y_queue = []

    # Loops until exception raised
    while True:
        # Document plz
        with g_frame_cond:
            if not g_frame_pred:
                g_frame_cond.wait()

            g_frame_pred = False

        image = g_frame.array  # convert to cv2 handleable format
        contours = get_contours(image)
        markers = []
        for contour in contours:
            cnt_area = cv2.contourArea(contour)
            if cnt_area < MIN_AREA:
                continue
            # Rect is [pointA, pointB, angle]
            rect = cv2.minAreaRect(contour)

            box = np.int0(cv2.boxPoints(rect))

            # Finds the area of the rectangle
            distances = [pt_distance(box[0], box[1]),
                         pt_distance(box[0], box[2]),
                         pt_distance(box[0], box[3])]
            distances.sort()
            rect_area = distances[0] * distances[1]
            if rect_area / cnt_area > ERROR + 1:
                continue

            # If the ratio isn't between 2.45 and 3.05, continue
            if abs((distances[1] / distances[0]) - V_MARKER_RATIO) > ERROR * 3:
                continue

            bounding = cv2.boundingRect(contour)
            # saves all markers to list with their orientations
            if abs(rect[-1]) > 45:
                markers.append((bounding, [box], "R"))
            else:
                markers.append((bounding, [box], "L"))

        # Sorts markers by x pos
        markers.sort(key=lambda tup: tup[0][0])
        right_rect, left_rect = None, None
        right_box, left_box = None, None
        if len(markers) >= 2:
            # removes single markers
            if markers[0][-1] == "L":
                markers.pop(0)
            if markers[-1][-1] == "R":
                markers.pop(-1)
            # "Only the first couple is interesting" ~ Kalisch
            if len(markers) >= 2:
                right_rect, right_box, _ = markers[0]
                left_rect, left_box, _ = markers[1]

        # If found a couple, draws the data on screen
        if right_rect and left_rect:
            cv2.drawContours(image, left_box, 0, BOX_COLOUR, 2)
            cv2.drawContours(image, right_box, 0, BOX_COLOUR, 2)
            y = T_HEIGHT * RESOLUTION[1] / (2 * math.tan(V_VIEW_ANGLE) *
                                            left_rect[-1])
            x = T_HEIGHT * RESOLUTION[1] / (2 * math.tan(V_VIEW_ANGLE) *
                                            right_rect[-1])
            cv2.putText(image, str(round(x, 2), right_rect[:2],
                        cv2.FONT_HERSHEY_SIMPLEX, 1, TEXT_COLOUR, 2))
            cv2.putText(image, str(round(y, 2), left_rect[:2],
                        cv2.FONT_HERSHEY_SIMPLEX, 1, TEXT_COLOUR, 2))

            frames_since += 1
            if int(time()) != last_sec:
                curr_fps = frames_since
                frames_since = 0
                last_sec = int(time())

            x_queue.append(x)
            y_queue.append(y)

            # If has enough points to use their average
            if len(x_queue) == QUEUE_SIZE + 1:
                x_queue.pop(0)
                y_queue.pop(0)

                x_avg = sum(x_queue) / QUEUE_SIZE
                y_avg = sum(y_queue) / QUEUE_SIZE
                dist = math.sqrt((x_avg ** 2 + y_avg ** 2 - 0.02) / 2)
                angle, target_x, target_y = target_details(x_avg, y_avg, dist)

                if ntapi.get_bool(ntapi.VISION_START_KEY, False):

                    left_prof, right_prof = motion_profiler(target_x,
                                                            target_y, angle)
                    ntapi.send_motion_profiles(left_prof, right_prof)

                data = [dist, math.degrees(angle), curr_fps]
                info = INFO_TEMPLATE.format(*data)

                cv2.putText(image, info, (0, RESOLUTION[1] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, TEXT_COLOUR, 2)
        else:
            x_queue = []
            y_queue = []

        cv2.imshow("contours", image)
        key = cv2.waitKey(1) & 0xFF  # if exit key is pressed

        if key == EXIT_KEY:  # exit
            raise KeyboardInterrupt
    # toggle_light(False)


if __name__ == MAIN:
    thread = Thread(target=acquire_frames)
    thread.start()

    # ntapi.init_and_wait()
    while True:
        try:
            main()
        except KeyboardInterrupt:
            break
        except Exception as e:  # o byle please forgive me
            print("died with {}\nrestarting...".format(e))

    g_running = False
