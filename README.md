![Betaflight](docs/assets/images/bf_logo.png)

[![Build Status](https://travis-ci.com/betaflight/betaflight.svg?branch=master)](https://travis-ci.com/betaflight/betaflight)

Betaflight is flight controller software (firmware) used to fly multi-rotor craft and fixed wing craft.

This is a customized Betaflight firmware that specifically targets on enabling SITL and Bi-copter research and development.

## Description

### Bi-copter version

This edited version of betaflight is dedicated for bi-copter implementation and debugging for personal use. Owned by Jack He.

For real flight setup(not sim), the board I used is [F4 V3 Plus](https://item.taobao.com/item.htm?spm=a1z09.2.0.0.52812e8dCAEObX&id=555909671335&_u=i360jks6b873), which supports 6 channel PWM output and three UART communication ports.
Use 
```
make target=OMNIBUSF4SD
```
for compiling.

WBUS is the same as SBUS so set it up in the configuration tool on the tab "Configuration/Receiver".

You need to enable the TIM_USE_SERVO on the PIN setup in the target.c file.

Copy and paste the following CLI command to enable M5 and M6 for servo output.
```
resource MOTOR 5 NONE
resource MOTOR 6 NONE
resource SONAR_TRIGGER NONE
resource SONAR_ECHO NONE
resource SERVO 1 A01
resource SERVO 2 A08
save
```

### Half-flip Setup
In Betaflight Configurator, set the CLI as
```
# Disable runway takeoff check especially when testing on test stand
set runway_takeoff_prevention=off
# Turn off RC interpretation for throttle since it needs to go reverse
set rc_interp_ch=RPY
```
The motor 3D needs to be turned off for BLHeli setup. Turn 3D back on after setting the ESC. In BLHeli software, set "Low RPM Power Protect" to off.


## OMNIBUSF4SD(OBSD)
### Flashing

For linux user only, if the configurator gives you DFU mode but not flashing when clicking "Flash firmware", try the following steps to add a new rule for user and let the system knows the DFU mode.

```
sudo emacs -nw /etc/udev/rules.d/45-stdfu-permissions.rules
```
Add the following script:
```
# DFU (Internal bootloader for STM32 MCUs)
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0664\
", GROUP="plugdev"
```
Then
```
sudo usermod -a -G plugdev <USERNAME>
```

## SITL simulation
### Configuring SITL

Following the README.md in `src/main/SITL/README.md`
Do not use the code here to read transmitor.

### Configuring joystick

Use the code [here](https://gist.github.com/JJJJJJJack/837a12689253c2f5ad29b3349068c2fc)

How-to-run: `node node emu-dx6-msp.js`

Configure the joystick using jstest-gtk. Install it with `sudo apt-get install jstest-gtk`.

Save js configuration using `sudo jscal-store /dev/input/js0`.



