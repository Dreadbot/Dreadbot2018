# Notes on SmartDashboard integration

See nt3656.py for code that reads values from networktables.

It reads values from roborio-3656-frc.local:/SmartDashboard/

    hsv_h_lo = ntproperty( '%s/vision_hsv_h_lo' % sdpath, 53.0, writeDefault=writeDefault )
    hsv_h_hi = ntproperty( '%s/vision_hsv_h_hi' %sdpath, 103.0 , writeDefault=writeDefault )
    hsv_s_lo = ntproperty( '%s/vision_hsv_s_lo' %sdpath,   0.0 , writeDefault=writeDefault )
    hsv_s_hi = ntproperty( '%s/vision_hsv_s_hi' %sdpath, 255.0 , writeDefault=writeDefault )
    hsv_v_lo = ntproperty( '%s/vision_hsv_v_lo' %sdpath, 100.0 , writeDefault=writeDefault )
    hsv_v_hi = ntproperty( '%s/vision_hsv_v_hi' %sdpath, 255.0 , writeDefault=writeDefault )
    vis_cam_brightness = ntproperty( '%s/vision_cam_brightness' %sdpath, 30, writeDefault = writeDefault )

DreadbotVisionEngine periodically:
  - reads from the above values (using the Nt3656 object.
  - It checks to see if the values read are valid. [0.0 - 255.0 ] values mostly and that lo <= hi
  - if valid and one or more of the values have changed from the current values
  -    it applies the newly changed values to the yellowboxgrip pipeline class
  - if the values are not valid
  -    the engine retains its previous values and ignores the invalid entries
  - if the values have changed
  -    saves a copy of the values to nt3656-current-vals.json file and internal data structure (dictionary)
  -    writes a feedback message to /vision_hsv_applied indicating the processing cycle on which the new values were applied.
  
  



# Refactor notes.

2018 we need to support multiple grip pipelines. Refactoring code to do this.

Probable design

BaseSensorPipeline class. Class that holds methods common to pipeline tasks
YelloboxSensor class. Class that is responsible for running the yellowbox detection over images.
AutolineSensor class. Class that is responsible for running the autoline (black) floor line detection over
                      images.
DreadbotVisionEngine class. Class that manages capture.

