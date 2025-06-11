# Structure 2: Joint control and feedback data (6 bytes total)
# First byte: control bits for negative/positive sign of joint positions
# First byte example:  00000101 -> Joint 2 negative, Joint 1 positive, Airsoft Trigger (Fire ON)
# First byte example:  00000010 -> Joint 2 positive, Joint 1 negative, Airsoft Trigger (Fire OFF)
# Position values: 0-135 (uint8)
uint8 control_bits      # Bit field: [bit1: airsoft_trigger, bit2: j1p_sign, bit3: j2p_sign, bit4-8: reserved]
                        # j1p_sign: 0=positive(+), 1=negative(-)
                        # j2p_sign: 0=positive(+), 1=negative(-)
uint8 j1p               # Joint 1 Position (0-135, sign in control_bits)
uint8 j1s               # Joint 1 Speed/Power (0-100 percent)
uint8 j2p               # Joint 2 Position (0-135, sign in control_bits)  
uint8 j2s               # Joint 2 Speed/Power (0-100 percent)
uint8 ap                # Airsoft Power (0-100 percent)
