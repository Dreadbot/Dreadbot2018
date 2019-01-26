#!/usr/bin/env python

from picamera import PiCamera
from time import sleep
import os,sys
from PIL import Image
import numpy

camera = PiCamera()

camera.resolution = (800, 600)

camera.start_preview()
camera.brightness = 50
sleep(2)
for i in range(2):
    sleep(0.5)
    camera.capture('/home/pi/Desktop/image%s.jpg' % i)

camera.stop_preview()

img = Image.open('/home/pi/Desktop/image0.jpg')
(width,height) = img.size
image = list(img.getdata())
image = numpy.array(image)
image3d = image.reshape((height,width,3))

new_img = img.resize((256,256))
new_img.show()
new_img.save('/home/pi/Desktop/i','png')
