#!/bin/bash  

echo "Executing assignment in Xterm windows"

roslaunch assignment0 grade_asgn0.launch &

sleep 2 
xterm -hold -e "rosrun assignment0 AutoGrade.py" 

sleep 2 
#Kill all ros nodes
killall -9 rosmaster


