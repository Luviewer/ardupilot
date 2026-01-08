1. cd tricopter 
2. ../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --add-param=mav.parm

3. 在启动ardupilot终端后：
STABILIZE> alt_hold
ALT_HOLD> rc 6 1500
ALT_HOLD> arm throttle
ALT_HOLD> arm throttle
ALT_HOLD> rc 3 1500
ALT_HOLD> rc 3 1700
ALT_HOLD> rc 3 1500