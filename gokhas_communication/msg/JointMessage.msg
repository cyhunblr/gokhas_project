uint8 control_bits
uint8 j1p
uint8 j1s
uint8 j2p
uint8 j2s
uint8 ap

# Structure 2: Joint control and feedback data (6 bytes total)
# First byte example:  00000000 -> bit3: 0 - Angle Control | bit2: 0 - Joint 2 Positive | bit1: 0 - Joint 1 Positive | bit0: 1 - Airsoft Trigger OFF
# First byte example:  00001111 -> bit3: 0 - Step Control  | bit2: 0 - Joint 2 Negative | bit1: 0 - Joint 1 Negative | bit0: 1 - Airsoft Trigger ON

# uint8 control_bits      Bit field: [bit0: airsoft_trigger, bit1: j1p_sign, bit2: j2p_sign, bit3-7: reserved]
#                         j1p_sign: 0=positive(+), 1=negative(-)
#                         j2p_sign: 0=positive(+), 1=negative(-)
# uint8 j1p               Joint 1 Position (0-135, sign in control_bits)
# uint8 j1s               Joint 1 Speed/Power (0-100 percent)
# uint8 j2p               Joint 2 Position (0-135, sign in control_bits)  
# uint8 j2s               Joint 2 Speed/Power (0-100 percent)
# uint8 ap                Airsoft Power (0-100 percent)