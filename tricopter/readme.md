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


## 说明
电机舵机安装顺序为右上、后、左上
pwm大于1500时，左上、右上、后都绕x轴正向转，也就是朝机体向后转
所有上旋翼是逆时针旋转
所有下旋翼是顺时针旋转