#!/usr/bin/env python

from __future__ import print_function

import argparse
import cv2
 
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True,
	help="path to the input image")
ap.add_argument("-c", "--cascade",
	default="haarcascade_frontalcatface.xml",
	help="path to cat detector haar cascade")
args = vars(ap.parse_args())

# load the input image and convert it to grayscale
image = cv2.imread(args["image"])
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
 
# load the cat detector Haar cascade, then detect cat faces
# in the input image
SCALE_FACTOR=1.05
detector = cv2.CascadeClassifier(args["cascade"])

print("Detecting")

def detectCats(scale):
    print( "trying scale=%4.2f" % scale )
    return detector.detectMultiScale(gray, scaleFactor=scale,
	minNeighbors=10, minSize=(75, 75))

for scaleidx in [1,2,3,4,5,6,7,8,9,10]:
    scale1 = SCALE_FACTOR - (scaleidx * 0.01)
    scale2 = SCALE_FACTOR + (scaleidx * 0.01)

    scale=scale1
    rects = detectCats(scale1)
    if len(rects)<1:
       scale=scale2
       rects = detectCats(scale2)
    if len(rects)>0:
       print( "Cat detected: scale=%4.2f" % scale )
       break
    else:
       print( "No cat detected. Trying next scale" )


print( rects )

# loop over the cat faces and draw a rectangle surrounding each
for (i, (x, y, w, h)) in enumerate(rects):
	cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
	cv2.putText(image, "Cat #{}".format(i + 1), (x, y - 10),
		cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
 
# show the detected cat faces
cv2.imshow("Cat Faces", image)
cv2.waitKey(0)
