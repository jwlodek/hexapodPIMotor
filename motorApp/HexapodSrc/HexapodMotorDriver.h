/*
FILENAME...   HexapodMotorDriver.h
USAGE...      Motor driver support for the Parker Hexapod series of controllers, including the Aries.

Mark Rivers
March 28, 2011

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

/** drvInfo strings for extra parameters that the Hexapod controller supports */
#define HexapodJerkString           "Hexapod_JERK"
#define HexapodReadBinaryIOString   "Hexapod_READ_BINARY_IO"
#define HexapodBinaryInString       "Hexapod_BINARY_IN"
#define HexapodBinaryOutString      "Hexapod_BINARY_OUT"
#define HexapodBinaryOutRBVString   "Hexapod_BINARY_OUT_RBV"

// Definitions of the MSTA word:
#define STATUS_DIRECTION    (0x0001)
#define STATUS_DONE         (0x0002)
#define STATUS_PLUS_LS      (0x0004)
#define STATUS_HOME_LS      (0x0008)
#define STATUS_POSITION     (0x0020)
#define STATUS_SLIP_STALL   (0x0040)
#define STATUS_HOME         (0x0080)
#define STATUS_PRESENT      (0x0100)
#define STATUS_PROBLEM      (0x0200)
#define STATUS_MOVING       (0x0400)
#define STATUS_GAIN_SUPP    (0x0800)
#define STATUS_COMM_ERR     (0x1000)
#define STATUS_MINUS_LS     (0x2000)
#define STATUS_HOMED        (0x4000)

class epicsShareClass HexapodAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  HexapodAxis(class HexapodController *pC, int axis);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setClosedLoop(bool closedLoop);

private:
  HexapodController *pC_;      /**< Pointer to the asynMotorController to which this axis belongs.
                                *   Abbreviated because it is used very frequently */
  char axisName_[10];      /**< Name of each axis, used in commands to Hexapod controller */ 
  unsigned char axisAutoOff_ = 0;      /**< AutoOff of each axis */ 
  int autoOffRetries_ = 0;           /**< AutoOffRetries */ 
  double pulsesPerUnit_;   /**< Pulses per engineering unit, which is what Hexapod controller uses */ 
  int flagsReg_;           /**< Address of the flags register */ 
  int limitsReg_;          /**< Address of the limits register */ 
  int encoderPositionReg_; /**< Address of the encoder position register */ 
  int theoryPositionReg_;  /**< Address of the theoretical position register */ 
  double encoderPosition_; /**< Cached copy of the encoder position */ 
  double theoryPosition_;  /**< Cached copy of the theoretical position */ 
  int currentFlags_;       /**< Cached copy of the current flags */ 
  int currentLimits_;      /**< Cached copy of the current limits */ 
  
friend class HexapodController;
};

class epicsShareClass HexapodController : public asynMotorController {
public:
  HexapodController(const char *portName, const char *HexapodPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  /* These are the methods that we override from asynPortDriver */
  asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask);
  
  /* These are the methods that we override from asynMotorDriver */
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  HexapodAxis* getAxis(asynUser *pasynUser);
  HexapodAxis* getAxis(int axisNo);

  
  /* These are the methods that are new to this class */
  asynStatus readBinaryIO();
  
protected:
  int HexapodJerk_;          /**< Jerk time parameter index */        
#define FIRST_Hexapod_PARAM HexapodJerk_
  int HexapodReadBinaryIO_;  /**< Read binary I/O parameter index */ 
  int HexapodBinaryIn_;      /**< Binary input parameter index */
  int HexapodBinaryOut_;     /**< Binary output parameter index */
  int HexapodBinaryOutRBV_;  /**< Binary output readback parameter index */
#define LAST_Hexapod_PARAM HexapodBinaryOutRBV_

#define NUM_Hexapod_PARAMS (&LAST_Hexapod_PARAM - &FIRST_Hexapod_PARAM + 1)

private:
  int binaryIn_;
  int binaryOutRBV_;
  int binaryInReg_;
  int binaryOutReg_;
  
friend class HexapodAxis;
};
