#!/bin/bash

# First make sure it is compiled
make TARGET=SITL

# Run betaflight program and wait for 2s before launching the transmitor node
(roslaunch rosbridge_server rosbridge_websocket.launch) & (sleep 3 && rosrun bicopter bicopter_node) & (sleep 8 && node tools/read_js_from_js/emu-dx6-msp.js) & (sleep 6 && ./obj/main/betaflight_SITL.elf)



