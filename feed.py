import cv2
from smbus2 import *
from picamera.array import PiRGBArray
from picamera import PiCamera
from gpiozero import LED

EXIT_KEY = ord("q")
light = LED("GPIO21")


def light_on():
    with SMBusWrapper(1) as bus:
        msg = i2c_msg.write(0x70, [0x00, 0xFF])	 # turn on i2c msg
        bus.i2c_rdwr(msg)


def light_off():
    with SMBusWrapper(1) as bus:
        msg = i2c_msg.write(0x70, [0x00, 0x00])	 # turn off i2c msg
        bus.i2c_rdwr(msg)


# light_on()
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 50
camera.iso = 270
camera.shutter_speed = 750
rawCapture = PiRGBArray(camera, size=(640, 480))
for frame in camera.capture_continuous(rawCapture, format="bgr",
                                       use_video_port=True):
    image = frame.array	         # convert to cv2 handleable format
    cv2.imshow("Raw", image)     # display raw
    key = cv2.waitKey(1) & 0xFF  # if exit key is pressed
    rawCapture.truncate(0)	     # clear stream for next image
    if key == EXIT_KEY:	         # exit
        break
# light_off()
