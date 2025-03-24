# state values
uint8 STEADY = 0
uint8 DISCHARGING = 1
uint8 CHARGING = 2

float64 percent_charge    # percent charge as a float64 between 0.0 and 100.0
uint8 state             # one of the state values defined above (uint8)
string state_name