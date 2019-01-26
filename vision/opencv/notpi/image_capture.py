#!/usr/bin/env python

from __future__ import print_function

import numpy as np
import cv2
import io

"""
Capture images when not on a Raspberry PI.
"""

from cameras import MyCamera
                
cap = cv2.VideoCapture(0)
camera = MyCamera( cap, 0 )

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord(' '):
        camera.capture( "/tmp/output.png" )
        print( "Wrote to /tmp/output.png" )
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
