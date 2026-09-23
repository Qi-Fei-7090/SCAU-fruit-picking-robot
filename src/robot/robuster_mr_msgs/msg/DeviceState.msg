# new version device state
int8    base_state
int8    control_mode             # 0 - idle,1 - APP, 2 - joystick
float32 temperature              # Internal temperature of car
float64 linear_velocity
float64 angular_velocity
bool    front_bumper
bool    rear_bumper
int16   front_light
int16   rear_light
float32 battery_percentage
float32 battery_current          # Current value, which can be used to detect the state of charge
bool    e_stop
int32   fault                    # error value
float64 uptime                   # time since start in min