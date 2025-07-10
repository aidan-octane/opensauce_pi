#!/bin/bash

echo Running!
# allow time to connect to wifi 
# sleep 20

while ! hostname -I | grep -q '\.'; do
    echo "Waiting for Wi-Fi..."
    sleep 3
done

echo "Connected to Wi-Fi!"

cd /home/racer/Programming/tomato_cannon_auto_aim

source /home/racer/Programming/tomato_cannon_auto_aim/venv/bin/activate

echo Sleep over! Opening 3 terminals:

sudo pigpiod

lxterminal -e "bash -c '/home/racer/Programming/tomato_cannon_auto_aim/venv/bin/python /home/racer/Programming/tomato_cannon_auto_aim/auto_aim_motors.py; echo \"Press Enter to close\"; read'" &

lxterminal -e "bash -c '/home/racer/Programming/tomato_cannon_auto_aim/venv/bin/python /home/racer/Programming/tomato_cannon_auto_aim/auto_aim_camera.py; echo \"Press Enter to close\"; read'" &

lxterminal -e "bash -c '/home/racer/Programming/tomato_cannon_auto_aim/venv/bin/python /home/racer/Programming/tomato_cannon_auto_aim/trigger_convention.py; echo \"Press Enter to close\"; read'" 
