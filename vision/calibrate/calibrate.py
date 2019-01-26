#!/usr/bin/env python

from __future__ import print_function

"""
Simple tool for calibrating the HSV filter values for finding a target.

Take an image with the PI camera. Wait for the operator to indentify
targets in the image. Suggest a set of HsV filters to identify the
targets.

Inspired by the blog entry here:

http://www.pyimagesearch.com/2015/03/09/capturing-mouse-click-events-with-python-and-opencv/
"""

import argparse
import cv2
import io
import picamera
import numpy
#from matplotlib import pyplot as plt

def click_and_crop():
    print("click_and_crop")

class DreadbotCalibrate(object):

    def __init__(self):
        self.camera = None
        self.refPt = []
        self.cropping = False
        self.target = "yellowbox"

    def getImage(self):
        if not(self.camera):
            self.camera = picamera.PiCamera()
            # TODO: read the cam res from a central location
            self.camera.resolution = (400,300)
            self.camera.brightness = 30
            
        camera = self.camera
        # Create the in-memory stream
        stream = io.BytesIO()
        camera.capture(stream, format='jpeg')
        data = numpy.fromstring( stream.getvalue(), dtype=numpy.uint8)
        image = cv2.imdecode(data, 1)
        return image

    def showHist(self, img):
        #Create a blank 300x300 black image
        plot = numpy.zeros((300, 300, 3), numpy.uint8)
        # Fill image with red color(set each pixel to red)
        plot[:] = (255, 255, 255)
        
        red = (0, 0, 255)
        green = (0, 255, 0)
        blue = (255, 0, 0)        
        cv2.namedWindow( "HSV Histogram", cv2.WINDOW_NORMAL )
        cv2.resizeWindow( "HSV Histogram", 600,600 )
        cv2.imshow( "HSV Histogram", plot)
        chans = cv2.split(img)
        colors = ( "h", "s", "v" )
        #plt.figure()
        #plt.title( "HSV Histogram" )
        #plt.xlabel( "val" )
        #plt.ylabel( "# of pixels" )
        cv2.putText(plot, "HSV Histogram", (5, 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, red, 1)


        xoff=25
        yoff=275
        colormap={
            "h": green,
            "s": red,
            "v": blue
        }
        for (chan, color) in zip( chans, colors):
            hist = cv2.calcHist( [chan], [0], None, [256], [0,256] )
            print( "%s" % hist )
            lastpnt=( xoff, yoff)
            for x in xrange( 0, 255):
                pnt=(xoff+x,yoff-int(hist[x][0]))
                #print( "%s -> %s" % (lastpnt,pnt) )
                cv2.line( plot, lastpnt, pnt, colormap.get( "color" ) )
                lastpnt = pnt
        """
            plt.plot( hist, color)
            plt.xlim( [0, 256] )
        """
        cv2.imshow( "HSV Histogram", plot )
        cv2.waitKey(0)

    def reject_outliers(self, data, m=2.0):
        return data[numpy.abs(data - numpy.mean(data)) < m * numpy.std(data)]
    
    def _reject_outliers(self, data, m = 2.):
        d = numpy.abs( data - numpy.median(data))
        mdev = numpy.median(d)
        s = d/mdev if mdev else 0.
        return data[s<m]

    def summarize(self, name, data ):
        print( "%s" % sorted(data) )
        print( "%s: count=%s, min=%s, max=%s, ave=%s, median=%s" %
               (name,
                data.size,
                numpy.min(data), numpy.max(data), numpy.mean(data), numpy.median(data) )
        )

    def analyze(self, roi):
        _roi = roi.copy()
        _roi = cv2.cvtColor( roi, cv2.COLOR_BGR2HSV )

        #self.showHist( _roi )

        hsv_h = []
        hsv_s = []
        hsv_v = []

        height, width, depth = _roi.shape
        print( "height = %s, width = %s, depth = %s" % ( height, width, depth ) )
        for hidx in xrange(0, height):
            for widx in xrange(0, width):
                pix = _roi[hidx,widx]
                hsv_h.append( pix[0] )
                hsv_s.append( pix[1] )
                hsv_v.append( pix[2] )
                
                #for didx in xrange(0, depth):
                #    pix = _roi[hidx,widx,didx]
                #    print( pix )

        self.summarize( "hue", numpy.array(hsv_h) )
        self.summarize( "sat", numpy.array(hsv_s) )
        self.summarize( "val", numpy.array(hsv_v) )
                
        self.summarize( "hue",
                        self.reject_outliers(numpy.array(hsv_h)) )
        self.summarize( "sat",
                        self.reject_outliers(numpy.array(hsv_s)) )
        self.summarize( "val",
                        self.reject_outliers(numpy.array(hsv_v)) )
        
        


    def calibrate(self, image=None):
        self.refPt = []

        if not(image):
            self.img = self.getImage()
        else:
            # Load the image
            self.img = cv2.imread(image)
            
        clone= self.img.copy()

        cv2.namedWindow("Calibrate", cv2.WINDOW_NORMAL)
        cv2.resizeWindow( "Calibrate", 800,600 )        
        cv2.setMouseCallback("Calibrate",self.click_and_crop)
        
        
        
        while True:
            cv2.imshow( "Calibrate", self.img )

            key = cv2.waitKey( 100 ) & 0xFF

            if key == ord("r"):
                self.img = clone.copy()

            if key == ord("c"):
                break

            if key == ord("q"):
                return            

        if len(self.refPt) > 1:
            refPt = self.refPt
            roi = clone[ refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]

            bigroi = cv2.resize( roi, (0,0), fx=4.0, fy=4.0 )
            cv2.imshow("ROI", bigroi)
            cv2.waitKey(0)

            self.analyze( roi )
                    

    def click_and_crop(self, event, x,y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.refPt = [(x,y)]
            self.cropping = True

        elif event == cv2.EVENT_LBUTTONUP:
            self.refPt.append((x,y))
            cropping = False

            refPt = self.refPt
            cv2.rectangle( self.img, refPt[0], refPt[1], (0,255,0), 2)
            cv2.imshow( "Calibrate", self.img )
            print( "%s" % refPt )            
            cv2.waitKey( 0 )

    
        
                    
            
def main():
    ap = argparse.ArgumentParser( description="Utility to calibrate HSV values." )
    ap.add_argument( "-i", "--image", default=None, help="Image to use for calibration." )
    args=ap.parse_args()
    
    dbc = DreadbotCalibrate()
    dbc.calibrate(image=args.image)

if "__main__" == __name__:
    main()


        
