#!/usr/bin/env python

"""
V2 of the sensor prototype. 

Supports multiple pipeline processing executions per image frames. Allows us to detect multiple 
objects at the same time.
"""


from __future__ import print_function

import argparse
import time
import imutils

"""
Dreadbot sensor processor loop. 

Runs continuously.
Monitors PI camera.
Detects yellowbox target blobs.
Communicates back to the roborio via NetworkTables.

TODO: Ask the drive team programmers to notify us when we are in auton mode. 
Set /SmartDashboard/yellowbox_mode = "on" when auton starts
Set /SmartDashboard/yellowbox_mode = "off" when auton stops

Doing this will put the pi into continous image capture mode, so we can save the pictures off for later
analysis and possible training.
"""

import io
import os
import os.path

import cv2
import numpy
import math
import time
import datetime
from enum import Enum
from threadpool import ThreadPool

from utils import saveImageTask

userhome = os.path.expanduser( "~" )
exec(open(os.path.join( userhome, "dreadbots_config.py")).read())

debug = False
pool = ThreadPool(5)

# Import the methods and classes from utils.py into this namespace
from utils import *

class BaseSensorPipeline(object):
    """Each new sensor should inherit from this super class, it implements some methods common to
       processing GRIP pipelines."""

    def __init__(self, name, pipeline):
        self.name = name
        self.pipeline = pipeline
        self.engine = None
        self.targets = self.realtargets = {}
        """ target_zone: A filter band of where we expect the blobs to 
              appear in the image expressed in a percentage.
              0.25 indicates we expect the target blobs to appear in the
              quarter above and below the center of the image. """
        self.target_zone = (0.0, 1.0)  # Find targets in the entire image.
        self.color = white = (255,255,255)     # Default color used to annotate the image with objects identified.
        self.realblobcounter = 0

    def setengine(self, engine):
        self.engine = engine

    def _process(self, image):
        self.pipeline.process( image )

    def process(self, image):
        debug = self.engine.debug

        def getOutput( pipeline, find_output_meth_name, flist):
            if hasattr( pipeline, find_output_meth_name):
                objs = getattr( pipeline, find_output_meth_name, None)
                if objs:
                    flist.extend( objs )
                    if debug:
                        print( "%s has a hit." % find_output_meth_name )
            
        
        if self.engine and self.engine.debug:
            print( "enter process %s" % self.__class__ )
        #self.pipeline.process( image )
        self._process( image )
        features = { 'lines': [], 'contours': [], 'blobs': [] }
        blobs = []
        if hasattr( self.pipeline, "find_blobs_output" ):
            features[ 'blobs' ] = self.pipeline.find_blobs_output

        contours = []
        if hasattr( self.pipeline, "find_contours_output" ):
            # scoop out contours if you have them
            features[ 'contours' ] = self.pipeline.find_contours_output


        lines = []
        getOutput( self.pipeline, "filter_lines_output" , lines )
        getOutput( self.pipeline, "find_lines_0_output" , lines )
        getOutput( self.pipeline, "filter_lines_1_output" , lines )
        features[ 'lines' ] = lines
        """
        if hasattr( self.pipeline, "filter_lines_output" ):
            features[ 'lines' ] = self.pipeline.find_lines_output
        """
        if self.engine and self.engine.debug:
            print( "features[ 'lines' ]=%s" % len( features[ 'lines' ] ) )


        return features

    def updateRobot(self, engine):
        """Child classes should override this method."""

        self.engine = engine

        assert "Child classes should override this method."

    def analyze(self, engine, features):
        self.engine = engine

        assert "Child classes should override this method."

    def applyNettableVals(self, nt3556):
        """Each child should implement it's own version of this method to apply values important to it to it's sensor pipeline."""

        assert "Child classes should override this method."

    def status(self):
        # Report back to the engine any informational / debug information for inclusion in the status.txt file

        return {}

    def filter(self, blobs=[], contours=[], lines=[]):
        """Use the target_zone to filter blobs that are likely not of interest."""

        res=self.engine.getRes()
        center=self.engine.cp()
        top_y=self.target_zone[0]*res[1]
        bot_y=self.target_zone[1]*res[1]

        fblobs = []
        if self.engine.blobsupport:
            # blob support activity now optional
            for b in blobs:
                if b.size>self.minrad:            
                    if b.pt[1] >= top_y and b.pt[1] <= bot_y:
                        fblobs.append( b )

        fcontours = []
        for carr in contours:
            c = Contour( carr )
            if c.area>self.minarea:
                if c.cy >= top_y and c.cy <= bot_y:
                    fcontours.append( c )

        # Filter lines that are less than 40% of the horizontal res
        minlength = self.engine.getRes()[0] * 0.40
        flines=[]
        for line in lines:
            (lx1,ly1,lx2,ly2) = (line.x1, line.y1, line.x2, line.y2)
            #mx = abs(lx2-lx1)/2
            my = min(ly1,ly2) + abs(ly2-ly1)/2

            #print( "my=%s top_y=%s bot_y=%s ly1=%s ly2=%s" % (my, top_y, bot_y, line.y1, line.y2) )
            print( "line.length: %s > %s" % (line.length(), minlength) )
            if my >= top_y and my <= bot_y and line.length()>minlength:
                flines.append( line )

                
        if self.engine and self.engine.debug:
            print( "lines=%s flines=%s" % ( len(lines), len(flines) ) )

        self.targets[ "blobs" ] = fblobs
        self.targets[ "contours" ] = fcontours
        self.targets[ "lines" ] = flines

        return self.targets
        



class DreadbotVisionEngine(object):
    """Engine, this is the main processing loop that periodically captures camera images and
       executes all the sensors / pipelines we are using to detect objects."""

    def __init__(self, sensors,  debug=False, lowexposuremode=False, logfails=False, cameramode="picamera"):
        """@parms sensors = dictionary of sensors to watch
           @parms pipeline = instance of a grip pipeline
           @parms debug    = run the sensor in debug mode, showing object detected in images.
           @parms logfails = If we detect things in one image, log subsequent cycles where the objects disappear.
        @parms nocamera = Allow the sensor code to run without camera input. It will fetch an image from /var/tmp/images/currentImage/raw.jpg and perform object detection on that.
        """
        self.blobsupport = False  # Use the pipeline's blob detection.
        self.updateCurrentImageInterval = 1.0 # number of seconds to encode a new jpg image for the web interface / GUI.
        self.imagedir = imagedir # "images/" take the value from dreadbots_config.py now.
        #self.pipeline = pipeline
        self.sensors = sensors

        for sk in self.sensors.keys():
            s = sensors[ sk ]
            s.setengine( self )
            
        
        self.camera = None
        self.res = ( 400, 300 )
        self.cameramode = cameramode
        self.__stillimage = None # Read the still image once in nocamera mode.
        self.lowexposuremode = lowexposuremode
        self.nt3656 = None
        self.initNetTables()
        self.lastNtRead = None

        myresolutions = [ (100,76), (200,152), (400, 304) ]
        self.res = myresolutions[2]
        if self.cameramode == "picamera" :
            # This code is allowed to fail when running on non-pi linux computers
            import picamera
            
            camera = picamera.PiCamera()
            camera.resolution = self.getRes()
            self.camera = camera
            self.initCamera()

        if self.cameramode == "nocamera":
            tmpcurimg = os.path.join(  tmpimagedir, "currentImage", "raw.jpg" )             
            print( "Reading image from %s" % (tmpcurimg) )

        if self.cameramode == "usbcamera":
            from imutils.video import VideoStream
            self.camera = VideoStream(src=0).start()


            
        #self.sd = NetworkTables.getTable('SmartDashboard')

        self.lastannotated = None
        self.lastraw = None
        self.targets = {
            "blobs": [],
            "contours": [],
            "lines": []
            }
        self.lastupdate = None
        self.counters = { "frames" : 0 }
        self.now = None
        
        self.debug = debug
        self.logfails = logfails
        self.rawCapture = None

        self.realtargets = {}
        self.realblobcounter = 0

        # Info in the protocol to send back
        self.suggest_no = 0
        self.active = False
        self.lastsave = None

        if self.camera and not(self.lowexposuremode):
            #self.camera.brightness = int( self.nt3656.vis_cam_brightness )
            pass
        self.estimatedDistance = -1
        self.focalLength = 328.0 # measured in pixels
        """
        self.yellowbox_mode = self.name == "yellowbox"
        self.goal_mode = self.name == "highgoal"
        """

    def updateStatus(self):
        """Write info to a status.txt file."""

        
        if not(self.lastupdate) or (self.now - self.lastupdate).seconds > 5:
            secs = 1
            if self.lastupdate:
                secs = (self.now - self.lastupdate).seconds
            frames = self.counters['frames']
            fps = (1.0 * frames) / secs

            sensors=[]
            for sensorname in self.sensors.keys():
                s = self.sensors[ sensorname ]

                stat = s.status()
                if len(stat)>0:
                    stat["name"]=sensorname
                    sensors.append( stat )

            with open( "status.txt", "w") as outf:
                outf.write( "%s\n" % self.now )
                outf.write( "%.3f fps [%s frames, %s secs]\n" % ( fps, frames, secs ) )
                for sensor in sensors:
                    outf.write( "%s\n" % sensor )
            self.counters['frames'] = 0
            self.lastupdate = self.now
            
        

    def initCamera(self):
        """Logic to initialize the camera should be set here. Note the brightness is set in the main processing loop should it be
          adjusted via nettables."""

        # Info about these settings can be found here.
        # http://picamera.readthedocs.io/en/release-1.10/api_camera.html
        if self.camera:
            if self.cameramode == "picamera":
                if self.lowexposuremode:
                    self.camera.resolution = self.getRes()
                    self.camera.framerate = 32
                    self.camera.shutter_speed = int( self.nt3656.shutter_speed )  # in microsecs
                    self.camera.awb_mode = 'off'
                    self.camera.awb_gains = (2,2)
                    self.camera.exposure_mode = 'off'
                else:
                    # TODO: to be able to toggle between bright and low exposure we would need to save the original camera statements to toggle.
                    self.camera.brightness = 60 # TODO: set from network tables
                    #self.camera.framerate = 30
                    self.camera.shutter_speed = int( 1000000.0 / 60.0 )
                    #self.camera.flash_mode = 'on'
                    #self.camera.contrast = -25
                print( "camera.resolution={res}".format( res=(self.camera.resolution)) )                    
            if self.cameramode == "usbcamera":
                pass
            

        
    def initNetTables(self):
        from nt3656 import Nt3656
        self.nt3656 = Nt3656()

    def isAutonomous(self):
        """Check network tables to see if the robot is using us."""

        # When the roborio is using set the smartdashboard yellowbox_mode var to on, otherwise we assume we are NOT use.
        active = None
        """
        TODO: work with the programing team so they tell us we are in auton mode.
        if self.yellowbox_mode:
            active = self.nt3656.yellowbox_mode
        else:
            active = self.nt3656.highgoal_mode
        """
        active = "on"

        return active == "on"

    def getImage(self):
        camera = self.camera

        if not(camera):
            if self.__stillimage == None:
                # read an image from /var/tmp/images/currentImage/raw.jpg instead of capturing from the camera
                tmpcurimg = os.path.join(  tmpimagedir, "currentImage", "raw.jpg" )
                self.__stillimage = cv2.imread( tmpcurimg )
                self.__stillimage = imutils.resize( self.__stillimage, width=self.res[0] )
            return self.__stillimage

        image = None
        if self.cameramode == "picamera":
            from picamera.array import PiRGBArray
        
            #image = cv2.imdecode(data, 1)

            #if not(self.rawCapture):
            self.rawCapture = PiRGBArray(camera)
            # use_video_port=True, from picamera docs. The image captured this way
            #  is of lower quality but faster. Seems to be roughly 5 times faster.
            camera.capture(self.rawCapture, format="bgr", use_video_port=True)
            image = self.rawCapture.array.copy()
            return image

        if self.cameramode == "usbcamera":
            self.rawCapture = self.camera.read()
            self.rawCapture = imutils.resize( self.rawCapture, width=self.res[0] )
            
            return self.rawCapture


    def getRes(self):
        res = self.res # (400,304)
        if self.cameramode == "picamera":
            if self.camera:
                res = self.camera.resolution

        return res
    #|

    def cp(self):
        """Get the center point of the FOV"""
        res= self.getRes()
        cx=int(res[0]/2)
        cy=int(res[1]/2)

        return (cx,cy)



    def estimateDistance(self, c1, c2):
        """Using simalrity estimate the distance of the goal."""

        tarWidthInches=9.0

        c1br = c1.br
        c2br = c2.br

        c1mpx = abs( c1br[1] - c1br[0])/2
        c2mpx = abs( c2br[1] - c2br[0])/2        
        
        perceivedWidthInPixels=abs( c1mpx - c2mpx )
        dist = (tarWidthInches * self.focalLength) / perceivedWidthInPixels

        self.estimatedDistance = dist
        return dist



    def success(self):
        """Unit test code, to determine if the last processimage was successful or not"""

        return len( self.targets.get( "blobs" , [] ) ) == 2 or \
               len( self.targets.get( "contours", [] ) ) == 2


    def saveImage(self, rawimage, annotated, imgdir=None):
        # queue the image to be written by a thread.
        _imgdir=imgdir if imgdir else self.imagedir 
        pool.add_task( saveImageTask, { "rawimage": rawimage, "annotated": annotated, "imgdir": imgdir } )
        
    def _saveImage(self, rawimage, annotated,imgdir=None):
        if not(imgdir):
            now = datetime.datetime.now()
            self.suggest_no += 1
            seconds = secofday( now )
            imgdir = os.path.join( self.imagedir, "%s" % now.year, "%02d" % now.month, "%02d" % now.day, "%s" % seconds )

        if not(os.path.exists( imgdir ) ):
            os.makedirs( imgdir )

        rawfile = os.path.join( imgdir, "raw.jpg" )
        annotatedfile = os.path.join( imgdir, "annotated.jpg" )

        cv2.imwrite( rawfile, rawimage )
        cv2.imwrite( annotatedfile, annotated )

        if self.debug:
            print( "images written to %s" % imgdir )

    def processimage(self, imagefile ):
        """Convenience method primarily for training"""
        self.lastannotated = cv2.imread( imagefile )

        return self._processimage( self.lastannotated )

    def analyze(self, features):

        if self.debug:
            #cv2.resizeWindow( "Analyze", 800, 600 )
            res=self.getRes()
            cv2.imshow( "Raw", self.lastraw ) # , res[0], res[1] )
            cv2.waitKey(1)

        self.lastannotated = self.lastraw.copy()
                
        # Iterate thru the sensors and have them analyze the features detected.
        for sensorname in self.sensors.keys():
            sensor = self.sensors[ sensorname ]
            sensor.analyze( self, features.get( sensorname, {} ) )

        # Capture the images for possible monitoring and or review later.
        if self.isAutonomous(): # and self.camera
            if self.camera:
                imgarr = self.rawCapture
                if self.cameramode == "picamera":
                    imgarr = self.rawCapture.array
                self.saveImage( imgarr, self.lastannotated )
                self.lastraw = imgarr

            else:
                self.saveImage( None, None )

    def updateRobot(self):
        
        # Iterate thru the sensors and have them update the robot with objects detected
        for sensorname in self.sensors.keys():
            sensor = self.sensors[ sensorname ]
            sensor.updateRobot( self )
            

        
    def _processimage(self, image ):        
        """Primary method for using grip pipelines for processing the image."""

        gtimer = Timer("GRIP Pipeline")
        atimer = Timer("Analyze Blobs")
        nttimer = Timer( "Read NetTable")

        if True: # self.nt3656.isConnected():
            nttimer.start()
            nt3656 = self.nt3656
            """
            self.pipeline.setHsv( [ nt3656.hsv_h_lo, nt3656.hsv_h_hi ],
                                  [ nt3656.hsv_s_lo, nt3656.hsv_s_hi ],
                                  [ nt3656.hsv_v_lo, nt3656.hsv_v_hi ] )
            """
            if self.debug:
                print( "lowexpusure_mode=%s" % self.lowexposuremode )
                if self.camera and self.cameramode == "picamera":
                    print( "shutter_speed=%s" % self.camera.shutter_speed )
                """
                import json
                with open( "nt3656-current-vals.json", "w" ) as outf:
                    outf.write( json.dumps( self.nt3656 ) )
                """

            """
            if self.debug:
                print("hsv_threshold_hue=%s" % self.pipeline.__hsv_threshold_hue)
                print("hsv_threshold_saturation=%s" % self.pipeline.__hsv_threshold_saturation)
                print("hsv_threshold_value=%s" % self.pipeline.__hsv_threshold_value)
            """
            if self.camera and self.cameramode == "picamera":
                if  not(self.lowexposuremode):
                    # self.camera.brightness = int( self.nt3656.vis_cam_brightness )
                    pass
                else:
                    #self.camera.shutter_speed = int( self.nt3656.shutter_speed )  # in microsecs
                    pass
                    
            nttimer.stop()

        gtimer.start()
        features = {}
        for sensorname in self.sensors.keys():
            sensor = self.sensors[ sensorname ]
            _features = sensor.process( image )

            features[ sensorname ] = _features
            
        gtimer.stop()

        atimer.start()
        self.analyze( features )
        atimer.stop()        

    def saveCurrentImage(self):
        """
        # For the gui (visionui) write the current image out for the casual observer.
        """
        lastannotated = self.lastannotated if self.lastannotated != None else self.lastraw
        if self.camera:
            self.saveImage( self.lastraw,
                            lastannotated,
                            os.path.join(  tmpimagedir, "currentImage" ) ) 
        #urimg =  "currentImage.jpg" )
        #cv2.imwrite(  curimg, self.lastannotated )

    def readNettable(self):
        """Read the values from network tables and apply them to the pipeline, if changed?"""
        
        for sensorname in self.sensors.keys():
            sensor = self.sensors[ sensorname ]

            sensor.applyNettableVals( self.nt3656 )

        self.lastNtRead = datetime.datetime.now()
        #sys.exit(1)
        
            
        
    def watch(self):
        """Watch the camera detecting Pipeline blobs as they appear. Main processing loop."""

        itimer = Timer("getImage")
        gtimer = Timer("GRIP Pipeline")
        atimer = Timer("Analyze Blobs")
        utimer = Timer("Update Robo RIO")

        #timers = [ itimer, gtimer, atimer, utimer ]
        timers = [ itimer, utimer ]
        lastimageupdate=None

        if self.debug:
            cv2.namedWindow( "Analyze", cv2.WINDOW_NORMAL )
            #cv2.resizeWindow( "Analyze", 800, 600 )     
        
        while True:
            self.now = datetime.datetime.now()
            try:
                # Keep the loop running despite a failure
                tw1=time.time()
                itimer.start()
                self.lastraw = self.getImage( )
                # Provide an updated image to potentially render in the web interface
                now=datetime.datetime.now()
                if not(lastimageupdate) or (now-lastimageupdate).seconds>self.updateCurrentImageInterval:
                    self.saveCurrentImage()
                    lastimageupdate=now
                
                itimer.stop()
                if self.debug:
                    print( type( self.lastraw ) )
                self._processimage( self.lastraw )
                if True: # not(self.debug): Turns out we want to see pictures and update the robot
                    utimer.start()
                    self.updateRobot()
                    utimer.stop()

                self.updateStatus()
                if not(self.lastNtRead) or (now-self.lastNtRead).seconds > 1.0:
                    # Apply network table values every 10 seconds.
                    self.readNettable()
                    
                if self.debug:
                    for t in timers:
                        t.show()
            except:
                if self.debug:
                    # Raise the exception in debug mode, continue running otherwise
                    raise
                print("Ignoring exception")
            finally:
                #if self.debug:
                self.counters['frames'] = self.counters.get('frames') + 1
                    


    
    

#from yellowboxgrip import GripPipeline

def main(sensorfactory):
    """
    @grippipelinefactory factory to construct a pipeline based on a target.
    """
    global debug
    
    print("Let the pi warm up. Sleeping (0.5 secs)")
    time.sleep(3) # Let things warm on the pie?
    ap = argparse.ArgumentParser( description="Dreadbot Sensor Prototype" )
    ap.add_argument( "--debug","-d", action="store_true", default=False,
                     help="Debug mode. Show the camera image and annotations" )
    ap.add_argument( "--logfails","-l", action="store_true", default=False,
                     help="Dilengently log frames where the targets disappear after a detection.")
    ap.add_argument( "--targets", "-t", default="",
                     help="Choose yellowbox and or autoline seperated by commas." )
    ap.add_argument( "--cameramode", "-c", default="picamera",
                     help="Select the camera to use for vision: picamera, usbcamera, nocamera are valid values." )
#changed default to True for lowexposure
    ap.add_argument( "--lowexposure", "-e", action="store_true", default=False,
                     help="Use low exposure mode instead of low brightness." )
    
    args = ap.parse_args()

    cammodes = [ "picamera","usbcamera","nocamera" ]
    if args.cameramode not in cammodes:
        print( "ERROR: Unrecognized camera mode: '%s'. Please specify one of the following: " )
        print ("    %s" % cammodes )
        ap.usage()
        return

    targetstr=args.targets
    targets=[]
    for t in targetstr.split(","):
        if len( t.strip() ):
            targets.append( t.strip() )

    sensors={}
    for t in targets:
        assert t in [ "yellowbox",  "autoline" ], "Invalid target '%s' must be either 'yellowbox' or 'autoline'"
        sensor = sensorfactory.getSensor( t ) 
        sensors[ t ] = sensor 

    #NetworkTables.initialize(server='roborio-3656-frc.local')
    debug = args.debug
    dve = DreadbotVisionEngine(
        sensors,
        debug=args.debug,
        lowexposuremode=args.lowexposure,
        logfails=args.logfails,
        cameramode=args.cameramode
    )

    try:
        dve.watch()
    finally:
        # Close the camera before the program exits.
        if dve.camera and dve.cameramode == "picamera":
            dve.camera.close()

#if "__main__" == __name__:
#    main()
        
