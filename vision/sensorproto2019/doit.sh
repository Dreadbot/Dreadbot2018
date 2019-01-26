#!/bin/bash

cd ~/dreadbots/sensorproto2018/

#./sensorproto2018.py --targets yellowbox,autoline $*
#./sensorproto2018.py --debug --targets yellowbox,autoline $*
./sensorproto2018.py --debug --targets yellowbox,autoline --cameramode=usbcamera $*
#./sensorproto2018.py --debug --targets autoline $*
