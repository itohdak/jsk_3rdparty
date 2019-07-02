#!/bin/bash

source /home/leus/app_manager_ws/devel/setup.bash # app manager

export GOOGLE_APPLICATION_CREDENTIALS=/etc/opt/jsk/robot/JSK-HRP2-16a6aaf1474e.json
export DIALOGFLOW_PROJECT_ID=hrp2-xhwxsc
echo $GOOGLE_APPLICATION_CREDENTIALS
echo $DIALOGFLOW_PROJECT_ID

# rosparam set /robot/name hrp2017v
rosparam set /robot/name pr2
rosparam set /robot/type hrp2jsknts
