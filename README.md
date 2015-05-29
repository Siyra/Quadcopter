# Quadcopter
Code to run on Raspberry PI for Quadcopter control

## Compile instructions
This code has been compiled on a Raspberry PI with Raspbian which has a kernel with PREEMPT_RT enabled,
the image for this can be downloaded here (there is also image for the Raspberry PI 2):

http://docs.emlid.com/Downloads/Real-time-Linux-RPi1/

For this to work, you will also need PIGPIO, which can be downloaded here:

http://abyz.co.uk/rpi/pigpio/

Also these two libraries are neccesary:
liblapack-dev
libblas-dev

## Ground control
Using for example QGroundControl you can use the console to send ASCII commands.

For now it accepts the following commands:

START - starts the control loops and starts the motors

STOP - stops the control loops

SETPOINT throttle 0 yaw 0 pitch 0 roll 0 - Sets the setpoint values for the quadcopter

SETPID yaw kp 1 kd 1 ki 1 - Sets the PID constants for the yaw controller

SETPID stab kp 1 kd 1 ki 1 - Sets the PID constants for pitch and roll for the stability controller

SETPID rate kp 1 kd 1 ki 1 - Sets the PID constants for pitch and roll for the rate controller

You can also use each parameter seperately, for example:

SETPOINT pitch 0.3

or

SETPID stab kd 1.3
