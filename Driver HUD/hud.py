import cv2
import numpy as np
import math
from decimal import Decimal


EXIT_KEY = 'q'
ERROR = 0.15            # error margin allowed while detecting a ball
CONST = 392             # a camera constant for Jonathan's Laptop
BALL_SIZE = 0.25        # Cargo's diameter in cm
TEXT_OFFSET = 3         # offset for distance text
FACTOR = 40             # scaling amount for the crosshair size
COLOR = (255, 0, 0)     # crosshair color
HOLE_SIZE = 0.41        # size in cm of hatch-panel hole
MARK_INTERVAL_FACTOR = 3    # scaling amount for the distance between lines


def draw_crosshair(pic):
    # draws crosshair
    height, width = pic.shape[:2]
    cross_len = height/FACTOR
    cv2.line(pic, (width / 2, height / 2 - cross_len),
             (width / 2, height / 2 + cross_len), COLOR, 2)
    cv2.line(pic, (width / 2 - cross_len, height / 2),
             (width / 2 + cross_len, height / 2), COLOR, 2)
    # draws distance markings
    base = width/2 + MARK_INTERVAL_FACTOR * cross_len
    for i in range(1, 5):
        mark_h = CONST * HOLE_SIZE / i
        cv2.line(pic, (base, int((height - mark_h) / 2)),
                 (base, int((height + mark_h) / 2)), COLOR, 2)
        base += MARK_INTERVAL_FACTOR * cross_len


def nothing(_):
    pass


# calibration sliders for manual color calibration
cv2.namedWindow('Sliders')
cv2.createTrackbar('l1', 'Sliders', 0, 255, nothing)
cv2.createTrackbar('l2', 'Sliders', 0, 255, nothing)
cv2.createTrackbar('l3', 'Sliders', 0, 255, nothing)
cv2.createTrackbar('h1', 'Sliders', 255, 255, nothing)
cv2.createTrackbar('h2', 'Sliders', 255, 255, nothing)
cv2.createTrackbar('h3', 'Sliders', 255, 255, nothing)

cv2.namedWindow('Morph')
cv2.createTrackbar('kernel type', 'Morph', 0, 2, nothing)
cv2.createTrackbar('kernel size', 'Morph', 0, 20, nothing)
cv2.createTrackbar('operation', 'Morph', 0, 4, nothing)
cv2.createTrackbar('iterations', 'Morph', 0, 10, nothing)

cap = cv2.VideoCapture(0)
while cap.isOpened() and cv2.waitKey(1) != ord(EXIT_KEY):
    l1 = cv2.getTrackbarPos('l1', 'Sliders')
    l2 = cv2.getTrackbarPos('l2', 'Sliders')
    l3 = cv2.getTrackbarPos('l3', 'Sliders')
    h1 = cv2.getTrackbarPos('h1', 'Sliders')
    h2 = cv2.getTrackbarPos('h2', 'Sliders')
    h3 = cv2.getTrackbarPos('h3', 'Sliders')

    _, frame = cap.read()
    # filter frame to a set color range
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([l1, l2, l3]), np.array([h1, h2, h3]))

    ker_type = cv2.getTrackbarPos('kernel type', 'Morph')
    ker_size = cv2.getTrackbarPos('kernel size', 'Morph')
    op = cv2.getTrackbarPos('operation', 'Morph')
    itr = cv2.getTrackbarPos('iterations', 'Morph')
    # activates morphology
    if min(ker_size, ker_type, op, itr) > 0:
        if ker_type == 1:
            ker = cv2.getStructuringElement(cv2.MORPH_RECT,
                                            (ker_size, ker_size))
        else:
            ker = cv2.getStructuringElement(cv2.MORPH_CROSS,
                                            (ker_size, ker_size))

        if op == 1:
            mask = cv2.erode(mask, ker, iterations=itr)
        if op == 2:
            mask = cv2.dilate(mask, ker, iterations=itr)
        if op == 3:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, ker, iterations=itr)
        if op == 4:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, ker, iterations=itr)

    filtered = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow('filter', filtered)

    # detects candidate balls' contours
    _, contours, __ = cv2.findContours(mask, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_NONE)
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        # if ball size is large enough
        if w < 10 or h < 10:
            continue
        # aspect ratio filter
        aspect_ratio = float(w) / h
        if abs(1 - aspect_ratio) > ERROR:
            continue

        area_ratio = cv2.contourArea(contour) / (w * h)
        expected = math.pi / 4
        error = (expected - area_ratio) / expected
        # if contour area matches ball
        if abs(error) <= ERROR:
            # draw markings
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            distance = Decimal((CONST * BALL_SIZE) / h)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, "d=" + str(round(distance, 2)) + "m",
                        (x, y - TEXT_OFFSET), font, 1, (0, 255, 0))

    draw_crosshair(frame)
    cv2.imshow('Orange', frame)
cap.release()
