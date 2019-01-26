import cv2

class MyCamera(object):
    """A picamera like interface for capturing images."""
    
    def __init__(self, camera, camera_port):
        self.camera = camera
        self.camera_port = camera_port
        self.ramp_frames = 30 # number of frames to discard to adjust to light

    def _getImage(self, camera):
        retval, im = camera.read()
        return im

    def rawcapture(self, camera=None):
        if not(camera):
            camera=self.camera
        # Adjust to the light
        for idx in xrange(self.ramp_frames):
            self._getImage( camera )

        # Now we take an image
        camera_capture = self._getImage( camera )

        return camera_capture
            
    def capture(self, stream, format=None):
        closeCamera = not(self.camera)            
        camera = self.camera
        try:
            if not(camera):
                camera = cv2.VideoCapture(self.camera_port)

            camera_capture = self.rawcapture(camera )
            cv2.imwrite( stream, camera_capture )
                
        finally:
            if closeCamera and camera:
                # Close the camera so we can capture an image later
                del(camera)
#|
