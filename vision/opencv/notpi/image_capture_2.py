#!/usr/bin/env python

from __future__ import print_function

import numpy as np
import cv2
import io
import datetime

"""
Capture images when not on a Raspberry PI.
"""

from cameras import MyCamera

                
class FaceDetector(object):
    def __init__(self, cascade="../haarcascade_frontalcatface.xml"):
        self.scale_factor = 1.05
        self.detector = cv2.CascadeClassifier(cascade)
        self.image = None
        self.rects = ()

    def _detectCats(self,scale, gray):
        print( "trying scale=%4.2f" % scale )
        if scale<1.01:
            return ()
        return self.detector.detectMultiScale(gray, scaleFactor=scale,
                minNeighbors=10, minSize=(75, 75))
        
    def detectCats(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        found=False
        for scaleidx in [0,1,2,3,4,5,6,7,8,9,10]:
            scale1 = self.scale_factor - (scaleidx * 0.01)
            scale2 = self.scale_factor + (scaleidx * 0.01)

            scale=scale1
            rects = self._detectCats(scale1, gray)
            if len(rects)<1 and scale1!=scale2:
                scale=scale2
                rects = self._detectCats(scale2, gray)
            if len(rects)>0:
                print( "Cat detected: scale=%4.2f" % scale )
                self.scale_factor = scale
                self.rects = rects
                self.image = image
                found=True
                break
            else:
                print( "No cat detected. Trying next scale" )

        return found

    def show(self):
        image = self.image
        rects = self.rects
        if True : # self.image:
            print(rects)

            # loop over the cat faces and draw a rectangle surrounding each
            for (i, (x, y, w, h)) in enumerate(rects):
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(image, "Cat #{}".format(i + 1), (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
 
            # show the detected cat faces
            cv2.imshow("Cat Faces", image)
            cv2.waitKey(0)
        
                
cap = cv2.VideoCapture(0)
camera = MyCamera( cap, 0 )
fd = FaceDetector()


lastDetect=None
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('frame',gray)

    now = datetime.datetime.now()
    if not(lastDetect) or (now-lastDetect).seconds > 1:
        lastDetect=now
        img = camera.rawcapture()
        found = fd.detectCats(img)
        if found:
            fd.show()
    

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
