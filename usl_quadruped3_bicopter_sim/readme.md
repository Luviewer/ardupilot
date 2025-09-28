修改日志：

[20250721]：
重构了代码，将不同步态解耦，所有步态都基于`AP_QuadRuped_Base`这个基类，`AP_QuadRuped_trash`为旧的四足代码
仿真空间文件夹在`qruped_sim`, 以后的仿真步骤为
1. 进入仿真空间文件夹`cd qruped_sim`
2. `../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --add-param=usl_quadruped3_bicopter.param`
