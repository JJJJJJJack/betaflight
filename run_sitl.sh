#!/bin/bash

# Run betaflight program and wait for 2s before launching the transmitor node
./obj/main/betaflight_SITL.elf & (sleep 2 && node tools/read_js_from_js/emu-dx6-msp.js)



