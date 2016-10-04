TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library

LIBRARY_IOC += HexapodMotor

# motorRecord.h will be created from motorRecord.dbd
# install devMotorSoft.dbd into <top>/dbd
DBD += HexapodMotorSupport.dbd

# The following are compiled and added to the Support library
HexapodMotor_SRCS += HexapodMotorDriver.cpp

HexapodMotor_LIBS += motor
HexapodMotor_LIBS += asyn
HexapodMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

