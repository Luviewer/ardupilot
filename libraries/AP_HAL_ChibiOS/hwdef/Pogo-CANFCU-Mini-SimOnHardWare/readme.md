# 编译HIL arduplane 固件
./Tools/scripts/sitl-on-hardware/sitl-on-hw.py --board Pogo-CANFCU-Mini-SimOnHardWare --vehicle plane --simclass QuadPlane --frame quadplane-tilttri --defaults ./Tools/autotest/default_params/quadplane-tilttri.parm --upload


# 编译HIL arducopter 固件
./Tools/scripts/sitl-on-hardware/sitl-on-hw.py --board Pogo-CANFCU-Mini-SimOnHardWare --vehicle copter --simclass MultiCopter --upload

参考链接：[https://ardupilot.org/dev/docs/sim-on-hardware.html]