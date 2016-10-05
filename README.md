# hexapodPIMotor

Support for hexapods from PI - Germany.

## Communication

The communication with the controller is described in the documentation provided (./documentation/Software Doku PPMAC ISS 1_5.pdf). 

## Installing

### Clone this repository to your computer:
```
cd /path/to/desired/location/
git clone https://gitlab.nsls2.bnl.gov/luvizotto/hexapodPIMotor.git 
```

### Setup the locations of libs in [top]/configure/RELEASE:
```
ASYN=/usr/lib/epics
MOTOR=/epics/src/motor
```

### Compile it:
```
cd [top]
make
```

## How to use it

To make your IOC work with this driver, you need to follow a few steps: 
* Make your IOC point to the driver ([IOC top]/configure/RELEASE)
* Configure some variables in the file "st.cmd"
* Create DB files

### Making your IOC point to the driver

Add this line to your file [IOC top]/configure/RELEASE: 
```
MOTOR_HEX=/path/to/driver/directory
```

### Configuring variables in the file "st.cmd"

#### Motor Names
```
# MotorNameConfig(Motor Number, Motor Name)
MotorNameConfig(0, "H850X")
MotorNameConfig(1, "H850Y")
.
.
.
MotorNameConfig(17, "DUALC")
```

#### Motor Auto OFF Config
[Optional] Setting to automatically turn off the motor after a motion is complete.
```
# MotorAutoOffConfig(Motor Number, Motor Auto Off [0|1])
MotorAutoOffConfig(0, 1)
MotorAutoOffConfig(4, 1)
```
(The default is 0)

#### Create the Controller
```
# HexapodCreateController(PORT, ASYN PORT, number of axes, active poll period (ms), idle poll period (ms)
HexapodCreateController("HEX01", "HEX", 18, 100, 1000)
```

### Create DB files

Just like other motor drivers, you need to create a motor.substitutions file with all the definitions of your motors.

motor.substitutions example:
```
file "../../db/basic_motor.db"
{
pattern
{P,                                  M,     DTYP,     PORT, ADDR, DESC,            EGU, DIR,      VELO, VBAS, ACCL, BDST, BVEL, BACC,MRES, ERES, RDBD, UEIP, PREC, DHLM, DLLM, INIT}
{"XF\:08IDB\-OP\{HX\:1\-Ax\:X\}",    "Mtr", "asynMotor",HEX1, 0, "Base Hexapod X",mm,  Pos,      1,   0.,   1.,   0,    1,    .2,   1e-3, 1e-3, 1e-3, 1, 3,    0,    0,    ""}
{"XF\:08IDB\-OP\{HX\:1\-Ax\:Y\}",    "Mtr", "asynMotor",HEX1, 1, "Base Hexapod Y",mm,  Pos,      1,   0.,   1.,   0,    1,    .2,   1e-3, 1e-3, 1e-3, 1, 3,    0,    0,    ""}
{"XF\:08IDB\-OP\{HX\:1\-Ax\:Z\}",    "Mtr", "asynMotor",HEX1, 2, "Base Hexapod Z",mm,  Pos,      1,   0.,   1.,   0,    1,    .2,   1e-3, 1e-3, 1e-3, 1, 3,    0,    0,    ""}
.
.
.
{"XF\:08IDB\-OP\{HX\:DUAL\-Ax\:C\}", "Mtr", "asynMotor",HEX1, 17,"Dual Hexapod C",deg, Pos,      1,   0.,   1.,   0,    1,    .2,   1e-3, 1e-3, 1e-3, 1, 3,    0,    0,    ""}
}
```
