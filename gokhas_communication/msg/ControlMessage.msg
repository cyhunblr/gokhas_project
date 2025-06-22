uint8 comStatus
uint8 calibStatus  

# Structure 1: PC <-> STM32 control and status communication
# UART optimized with uint8 only

#uint8 comStatus          Communication status (0=inactive, 1=active)
#uint8 calibStatus        Calibration status (0=normal, 1=calibration_mode)