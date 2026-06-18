builtin_interfaces/Time timestamp

uint8 STATUS_OK = 0
uint8 STATUS_WARNING = 1
uint8 STATUS_ERROR = 2
uint8 STATUS_CRITICAL = 3

uint8 status
float32 cpu_usage
float32 ram_usage
float32 disk_usage
float32 battery_voltage
float32 battery_percentage
float32 temperature
string hardware_id
