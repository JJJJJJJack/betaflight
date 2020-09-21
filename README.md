![Betaflight](docs/assets/images/bf_logo.png)

[![Build Status](https://travis-ci.com/betaflight/betaflight.svg?branch=master)](https://travis-ci.com/betaflight/betaflight)

Betaflight is flight controller software (firmware) used to fly multi-rotor craft and fixed wing craft.

This fork differs from Baseflight and Cleanflight in that it focuses on flight performance, leading-edge feature additions, and wide target support.

## Events

| Date  | Event |
| - | - |


## Description

### Bi-copter version

This edited version of betaflight is dedicated for bi-copter implementation and debugging for personal use. Owned by Jack He.

### Configuring SITL

Following the README.md in `src/main/SITL/README.md`
Do not use the code here to read transmitor.

### Configuring joystick

Use the code [here][https://gist.github.com/JJJJJJJack/837a12689253c2f5ad29b3349068c2fc]

How-to-run: `node node emu-dx6-msp.js`

Configure the joystick using jstest-gtk. Install it with `sudo apt-get install jstest-gtk`.

Save js configuration using `sudo jscal-store /dev/input/js0`.
