local ODOMETRY = {}
ODOMETRY.id = 331
ODOMETRY.crc_extra = 91
ODOMETRY.fields = {
             { "time_usec", "<I8" },
             { "x", "<f" },
             { "y", "<f" },
             { "z", "<f" },
             { "q", "<f", 4 },
             { "vx", "<f" },
             { "vy", "<f" },
             { "vz", "<f" },
             { "rollspeed", "<f" },
             { "pitchspeed", "<f" },
             { "yawspeed", "<f" },
             }
return ODOMETRY
