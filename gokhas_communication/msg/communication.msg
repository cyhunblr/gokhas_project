# MOD STATES
bool  comStat       # if comStatus is 0, there is no comm
bool  modStat       # if modStatus is 0, there is in Manuel Mod
# RECIVE STATES
int16 rJ1p          # receive position of the Joint 1
int16 rJ1s          # receive speed of the Joint 1
int16 rJ2p          # receive position of the Joint 2
int16 rJ2s          # receive speed of the Joint 2
int16 rAp           # receive power of the Airsoft
bool  rAt           # receive trigger status of the Airsoft
# TRANSMIT STATES
int16 tJ1p          # transmit position of the Joint 1
int16 tJ1s          # transmit speed of the Joint 1
int16 tJ2p          # transmit position of the Joint 2
int16 tJ2s          # transmit speed of the Joint 2
int16 tAp           # transmit power of the Airsoft
bool  tAt           # transmit trigger status of the Airsoft
# CALIBRATION STATES
bool  cJ1           # do calibration for Joint 1
bool  cJ2           # do calibration for Joint 2
