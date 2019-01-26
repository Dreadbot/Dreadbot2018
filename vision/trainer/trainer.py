#!/usr/bin/python

from __future__ import print_function

import os
import os.path
import shutil
import cv2
import argparse

from geargrip import GripPipeline
from sensor import DreadbotSensor

class VisionTrainer(object):

    def __init__(self, sensor, interactive=True):
        self.sensor = sensor
        self.interactive = interactive

    def calibrate(self, imagefile):
        import subprocess
        args=[ "./calibrate.py", "-i", imagefile ]
        subprocess.call( args )
        print( " ".join(args) )

    def showfail(self, imagefile):
        self.sensor.debug = True
        self.sensor.processimage( imagefile )
        self.sensor.debug = False
        key = cv2.waitKey(0) & 0xFF

        if key == ord("c"):
            self.calibrate( imagefile )
            return

        if key == ord("q"):
            raise Exception("User aborted")
        
    def trainonimage(self, imagefile):
        print( "Training with %s" % imagefile )
        self.sensor.processimage( imagefile )
        if len(self.sensor.blobs) == 2:
            shutil.move( imagefile, os.path.join( self.succdir, os.path.basename( imagefile ) ) )
            return True
        else:
            print( "blobs=%s" % self.sensor.blobs )
            print( "%s fails" % imagefile )

            if self.interactive:
                self.showfail( imagefile )
            
            shutil.move( imagefile, os.path.join( self.faildir, os.path.basename( imagefile ) ) )
            return False

    def makedir(self, dirpath):
        if not os.path.exists(dirpath):
            os.makedirs(dirpath)

        return dirpath
            
    def train(self, image=None, imagedir="images"):
        self.imagedir = imagedir
        self.succdir = self.makedir( os.path.join( imagedir, "success" ) )
        self.faildir = self.makedir( os.path.join( imagedir, "failed" ) )
        
        print( "Training from images in %s" % imagedir)

        files = []
        if image:
            files = [ os.path.basename( image ) ]
        else:
            files = sorted( os.listdir(imagedir) )

        for f in files:
            if "." in f:
                self.trainonimage( os.path.join( imagedir, f) )


if __name__ == "__main__":
    ap = argparse.ArgumentParser( description="Dreadbots Vision Trainer" )
    ap.add_argument( "-i", "--image" , default = None, help="Specific image to examine" )
    ap.add_argument( "-n", "--noninteractive", action="store_true", default=False, help="Auto train without human review" )
    args = ap.parse_args()
    
    vt = VisionTrainer( DreadbotSensor( "gearsensor", GripPipeline() ), interactive=not(args.noninteractive) )

    vt.train(image=args.image)
                
            
