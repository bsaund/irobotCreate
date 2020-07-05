# irobotCreate
This is my library for controlling my irobot create (2 wheeled vacuum clearning robot).

## How to run:
1) 
```ssh raspi
source ~/venvs/irobotCreate/bin/activate
cd ~/code/irobotCreate/irobot/console_interfaces
python remote_serial.py --server
```
2) From a desktop computer run `tethered_drive.py`

## Install
1) Install ros on remote jetson nano and another controller desktop attached to irobot create
2) Clone this repo as a ros package on the nano(e.g. in `~/catkin_ws/src/`)
3) Clone this repo as a ros package on the control computer

## Jetson setup
If the Jetson wireless is slow, run `sudo iw dev wlan0 set power_save off`
https://forums.developer.nvidia.com/t/jetson-nano-not-responding-or-responding-very-slow-over-ssh/79530


## Acknowledgements
I build off of:
https://bitbucket.org/lemoneer/irobot/commits/all
