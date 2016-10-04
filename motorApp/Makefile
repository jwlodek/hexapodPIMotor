TOP = ..
include $(TOP)/configure/CONFIG

# MotorSrc is required for all motor record configurations.  It MUST appear
# 1st in this list of directories.

DIRS += MotorSrc

# All the following modules require ASYN.
ifdef ASYN

DIRS += HexapodSrc
HexapodSrc_DEPEND_DIRS = MotorSrc

endif

# Install the edl files
#DIRS += opi

DIRS += Db

include $(TOP)/configure/RULES_DIRS

