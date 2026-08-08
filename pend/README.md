# Pendulum SITL workspace

Start Gazebo in one terminal. This workspace is built for the repository's
ROS Noetic container; its host-side `devel/setup.bash` symlink points to
`/home/ros/uavros_ws` and is not usable directly on the ROS 2 Jazzy host:

```bash
cd /home/llw/Projects/uavros_ws
./scripts/docker-shell.sh
# Run the next command inside the container:
roslaunch uav_gazebo spawn.launch world_name:=pendulum_quadcopter
```

Start ArduCopter SITL in another terminal:

```bash
cd /home/llw/Projects/ardupilot
./Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f gazebo-iris \
    --use-dir=pend \
    --add-param-file=pend.param \
    --sitl-instance-args="--serial2=tcp:14557" \
    --console \
    --map
```

Use `--wipe-eeprom` once when applying `pend.param` to an existing workspace:

```bash
./Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f gazebo-iris \
    --use-dir=pend \
    --add-param-file=pend.param \
    --sitl-instance-args="--serial2=tcp:14557" \
    --wipe-eeprom \
    --console \
    --map
```

The Gazebo flight-dynamics plugin and `gazebo-iris` use UDP ports 9002 and
9003. The pole plugin connects as a TCP client to SITL SERIAL2 on port 14557
and sends MAVLink 2 `ODOMETRY` messages with system ID 42.

The current branch uses the Lua LQR controller in GUIDED mode. The native
PENDULUM mode is disabled at compile time, so do not use the older
`FLTMODE6=29` and `SCR_ENABLE=0` settings from `quad/pendulum.parm`.

`PLBL_*` parameters are created dynamically after the Lua script starts. On a
fresh EEPROM they use the defaults defined in `copter-pole-balance.lua`. After
the first boot they can be changed from MAVProxy and are stored in EEPROM, for
example:

```text
param show PLBL_*
param set PLBL_DEBUG 1
```

The current Lua defaults are:

```text
PLBL_ENABLE       1
PLBL_SYSID        42
PLBL_K_X          -0.316
PLBL_K_V          -0.946
PLBL_K_R          65.452
PLBL_K_RD         12.375
PLBL_ACC_MAX      2.0
PLBL_DIST_MAX     10.0
PLBL_DEBUG        0
PLBL_REF_VEL_MAX  0.2
PLBL_POLE_LEN     0.7
PLBL_TOP_OFS      0.08
PLBL_LAND_XY      0.12
PLBL_LAND_Z       0.08
PLBL_ACT_Z        1.0
PLBL_Z_VEL_MAX    1.0
PLBL_THROW_CH     8
PLBL_THROW_UP_MS  300
PLBL_THROW_DN_MS  300
PLBL_THROW_UP_V   2.0
PLBL_THROW_DN_V   -2.0
```
