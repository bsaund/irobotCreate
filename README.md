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


## Acknowledgements
I build off of:
https://bitbucket.org/lemoneer/irobot/commits/all
