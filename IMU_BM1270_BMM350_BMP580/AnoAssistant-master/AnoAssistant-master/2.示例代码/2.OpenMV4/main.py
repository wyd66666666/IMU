# Hello World Example
#
# Welcome to the OpenMV IDE! Click on the green run arrow button below to run the script!

import sensor, image, time
from pyb import UART
import AnoDTv8

uart = UART(3, 2500000)

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)   #
#sensor.set_transpose(True)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

low_threshold = (0, 50)
high_threshold = (205, 255)
_posx = 0
_fps = 0

while(True):
    clock.tick()                    # Update the FPS clock.
    img = sensor.snapshot()         # Take a picture and return the image.
    img.binary([low_threshold], to_bitmap=True)
    uart.write(AnoDTv8.AnoDataPackCamImg(img, 1, 1, _fps, 2,3,4,5,6,7,8,9,10))
    if uart.any() > 0:
        AnoDTv8.AnoRecvBuf(uart.read())
    if len(AnoDTv8.AnoTxBuf) > 0:
        uart.write(AnoDTv8.AnoTxBuf)
        AnoDTv8.AnoTxBuf = bytearray()
    #_len = uart.any()
    #if _len >0 :
    ##    print(_len)
    #    for i in range(_len):
    #        AnoDTv8.AnoReceiveOneByte(uart.read(1))
    #img.clear()
    #img.draw_cross(10+_posx, 10, color = (0, 255, 0))
    #uart.write(AnoDTv7.AnoDataPackCamImg(img, 1, 2, _fps, 2,3,4,5,6,7,8,9,10))
    #img.clear()
    #img.draw_cross(20+_posx, 20, color = (0, 255, 0))
    #uart.write(AnoDTv7.AnoDataPackCamImg(img, 0, 5, _fps, 2,3,4,5,6,7,8,9,10))
    #img.clear()
    #img.draw_cross(30+_posx, 30, color = (0, 255, 0))
    #uart.write(AnoDTv7.AnoDataPackCamImg(img, 1, 6, _fps, 2,3,4,5,6,7,8,9,10))
    #uart.write(AnoDTv7.AnoDataPackLogStr('test1234'))

    #_posx += 1
    #if _posx >= 40:
    #    _posx = 0
    #_fps = clock.fps()
    #print(_fps)                     # Note: OpenMV Cam runs about half as fast when connected
