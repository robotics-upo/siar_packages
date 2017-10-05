
Header header

# FW versions
string motor_fw_version
string battery_fw_version

# Motor status
int8 operation_mode
bool reverse
bool slow
bool fan
bool hard_stop
bool is_stopped # Indicates if there is one of the wheels on movement or not
uint8 hard_stop_time

uint16 lin_motor_pos
uint16 lin_motor_pot

float32 electronics_x
float32 width

nav_msgs/Odometry odom

int64 front_left_ticks
int64 front_right_ticks
int64 middle_left_ticks
int64 middle_right_ticks
int64 back_left_ticks
int64 back_right_ticks

float64 x
float64 y
float64 speed

# battery status
SiarBatteryMonitor elec_battery
SiarBatteryMonitor motor_battery
SiarPowerSupply power_supply

bool front_light
bool rear_light

#arm related
int16[5] herculex_position
uint8[5] herculex_temperature
uint8[5] herculex_status
uint8[5] herculex_torque
bool arm_panic

#auxiliar pins
uint8 aux_pins_values

#TODO: integrate the network status
#Nvip_status network_status

