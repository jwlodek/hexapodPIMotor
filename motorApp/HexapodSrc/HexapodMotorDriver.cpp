/*
FILENAME... HexapodMotorDriver.cpp
USAGE...    Motor driver support for the Parker Hexapod series of controllers, including the Aries.

Mark Rivers
March 4, 2011

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "HexapodMotorDriver.h"

/* --- Local data. --- */
int current_num_axis = 0;
char **Hexapod_axis = NULL;
unsigned char *Hexapod_axis_auto_off = NULL;

static const char *driverName = "HexapodMotorDriver";

/** Creates a new HexapodController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] HexapodPortName       The name of the drvAsynIPPPort that was created previously to connect to the Hexapod controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
HexapodController::HexapodController(const char *portName, const char *HexapodPortName, int numAxes, 
                             double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_Hexapod_PARAMS, 
                         asynUInt32DigitalMask, 
                         asynUInt32DigitalMask,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  static const char *functionName = "HexapodController";

  /* Connect to Hexapod controller */
  status = pasynOctetSyncIO->connect(HexapodPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s:%s: cannot connect to Hexapod controller\n",
      driverName, functionName);
  }
  writeController();
  // Wait a short while so that any responses to the above commands have time to arrive so we can flush
  // them in the next writeReadController()
  epicsThreadSleep(0.5);

  // Create the axis objects
  for (axis=0; axis<numAxes; axis++) {
    new HexapodAxis(this, axis);
  }


  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new HexapodController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] HexapodPortName       The name of the drvAsynIPPPort that was created previously to connect to the Hexapod controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int HexapodCreateController(const char *portName, const char *HexapodPortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  new HexapodController(portName, HexapodPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  return(asynSuccess);
}

/*****************************************************/
/* Configure a motor name                            */
/* MotorNameConfig()                                 */
/*****************************************************/
extern "C" int
MotorNameConfig(int card,        /* motor number being configured */
             const char *name)   /* motor name */
{
    current_num_axis++;
    Hexapod_axis = (char**)realloc(Hexapod_axis, (current_num_axis + 1)*sizeof(*Hexapod_axis));
    Hexapod_axis[card] = (char*)malloc(sizeof(name));
    strcpy(Hexapod_axis[card], name);
    return(asynSuccess);
}


/*****************************************************/
/* Configure a motor auto off setting                */
/* MotorAutoOffConfig()                              */
/*****************************************************/
extern "C" int
MotorAutoOffConfig(int card,        /* motor number being configured */
             const unsigned char autoOff)   /* Auto off setting */
{
    // setup size of auto off config array
    if(!Hexapod_axis_auto_off)
    {
      Hexapod_axis_auto_off = (unsigned char*)realloc(Hexapod_axis_auto_off, (current_num_axis + 1) * sizeof(unsigned char));
      for(int i = 0; i < current_num_axis; ++i)
        Hexapod_axis_auto_off[i] = 0;
    }
    Hexapod_axis_auto_off[card] = autoOff;
    return(asynSuccess);
}


/** Returns a pointer to an HexapodMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
HexapodAxis* HexapodController::getAxis(asynUser *pasynUser)
{
  return static_cast<HexapodAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an HexapodMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
HexapodAxis* HexapodController::getAxis(int axisNo)
{
  return static_cast<HexapodAxis*>(asynMotorController::getAxis(axisNo));
}


/** Called when asyn clients call pasynInt32->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * If the function is motorSetClosedLoop_ then it turns the drive power on or off.
  * If the function is HexapodReadBinaryIO_ then it reads the binary I/O registers on the controller.
  * For all other functions it calls asynMotorController::writeInt32.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write. */
asynStatus HexapodController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  HexapodAxis *pAxis = getAxis(pasynUser);
  static const char *functionName = "writeInt32";
  
  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setIntegerParam(pAxis->axisNo_, function, value);
  
  /* Call base class method */
  status = asynMotorController::writeInt32(pasynUser, value);
  
  /* Do callbacks so higher layers see any changes */
  callParamCallbacks(pAxis->axisNo_);
  if (status) 
    asynPrint(pasynUser, ASYN_TRACE_ERROR, 
        "%s:%s: error, status=%d function=%d, value=%d\n", 
        driverName, functionName, status, function, value);
  else    
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
        "%s:%s: function=%d, value=%d\n", 
        driverName, functionName, function, value);
  return status;
}

/** Called when asyn clients call pasynFloat64->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * If the function is HexapodJerk_ it sets the jerk value in the controller.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * For all other functions it calls asynMotorController::writeFloat64.
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus HexapodController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  HexapodAxis *pAxis = getAxis(pasynUser);
  static const char *functionName = "writeFloat64";
  
  
  /* Set the parameter and readback in the parameter library. */
  status = setDoubleParam(pAxis->axisNo_, function, value);
  
  status = asynMotorController::writeFloat64(pasynUser, value);
  
  /* Do callbacks so higher layers see any changes */
  pAxis->callParamCallbacks();
  if (status) 
    asynPrint(pasynUser, ASYN_TRACE_ERROR, 
        "%s:%s: error, status=%d function=%d, value=%f\n", 
        driverName, functionName, status, function, value);
  else    
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
        "%s:%s: function=%d, value=%f\n", 
        driverName, functionName, function, value);
  return status;
}

/** Called when asyn clients call pasynUInt32Digital->write().
  * Writes a single bit to one of the Hexapod binary output registers. 
  * This function is limited to writing a single bit, because we use the BIT command.
  * It writes to least significant bit that is set in the mask.
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write.
  * \param[in] mask Mask value to use when writinging the value. */
asynStatus HexapodController::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask)
{
  asynStatus status;
  // Not implemented on the Hexapods

  return(status);
}

/** Reads the binary input and binary output registers on the Hexapod.
  * Sets the values in the parameter library.
  * Keeps track of which bits have changed.
  * Calls any registered callbacks for this pasynUser->reason and address. */ 
asynStatus HexapodController::readBinaryIO()
{
  asynStatus status;
  // Not implemented on the Hexapods

  return status;
}


// These are the HexapodAxis methods

/** Creates a new HexapodAxis object.
  * \param[in] pC Pointer to the HexapodController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
HexapodAxis::HexapodAxis(HexapodController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  asynStatus status;

  sprintf(axisName_, "%s", Hexapod_axis[axisNo]);
  if(Hexapod_axis_auto_off)
    axisAutoOff_ = Hexapod_axis_auto_off[axisNo];
  pulsesPerUnit_ = 0.001;
  setIntegerParam(pC_->motorStatusHasEncoder_, 1);
  callParamCallbacks();
}

asynStatus HexapodAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "moveAxis";

  if (relative) {
    sprintf(pC_->outString_, "%s REL=%f", axisName_, position * pulsesPerUnit_);
    status = pC_->writeController();
  } else {
    sprintf(pC_->outString_, "%s VAL=%f", axisName_, position * pulsesPerUnit_);
    status = pC_->writeController();
  }
  return status;
}

asynStatus HexapodAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status;
  // static const char *functionName = "homeAxis";

  sprintf(pC_->outString_, "%s HOMR", axisName_);
  status = pC_->writeController();
  return status;
}

asynStatus HexapodAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "moveVelocityAxis";

  // No support for moveVelocity in PI's current system
  return status;
}

asynStatus HexapodAxis::stop(double acceleration )
{
  asynStatus status;
  //static const char *functionName = "stopAxis";

  sprintf(pC_->outString_, "%s STOP", axisName_);
  status = pC_->writeController();
  return status;
}

asynStatus HexapodAxis::setPosition(double position)
{
  asynStatus status;

  /*sprintf(pC_->outString_, "%s RES %f", axisName_, position/pulsesPerUnit_);
  status = pC_->writeController();
  sprintf(pC_->outString_, "%s JOG REN", axisName_);
  status = pC_->writeController();*/
  return status;
}

asynStatus HexapodAxis::setClosedLoop(bool closedLoop)
{
  asynStatus status;

  if(closedLoop)
    sprintf(pC_->outString_, "%s MotorOn", axisName_);
  else
    sprintf(pC_->outString_, "%s MotorOff", axisName_);
  status = pC_->writeController();
  return status;
}

/** Polls the axis.
  * This function reads the controller position, encoder position, the limit status, the moving status, 
  * and the drive power-on status.  It does not current detect following error, etc. but this could be
  * added.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */
asynStatus HexapodAxis::poll(bool *moving)
{ 
  int homed, done, driveOn, limit, msta, gain_supp, mov;
  asynStatus comStatus;


  // Read the current encoder position
  sprintf(pC_->outString_, "%s RBV", axisName_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  encoderPosition_ = atof(pC_->inString_);
  setDoubleParam(pC_->motorEncoderPosition_,encoderPosition_/pulsesPerUnit_);

  // Read the current theoretical position
  sprintf(pC_->outString_, "%s VAL", axisName_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  theoryPosition_ = atof(pC_->inString_);
  setDoubleParam(pC_->motorPosition_, theoryPosition_/pulsesPerUnit_);

  // Read the current limit status
  sprintf(pC_->outString_, "%s HLS", axisName_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  limit = atoi(pC_->inString_);
  setIntegerParam(pC_->motorStatusHighLimit_, limit);

  sprintf(pC_->outString_, "%s LLS", axisName_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  limit = atoi(pC_->inString_);
  setIntegerParam(pC_->motorStatusLowLimit_, limit);

  // Read the MSTA word to check some of the bits
  sprintf(pC_->outString_, "%s MSTA", axisName_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  msta = atoi(pC_->inString_);

  limit     = (msta & STATUS_HOME)      > 0 ? 1 : 0;
  done      = (msta & STATUS_DONE)      > 0 ? 1 : 0;
  homed     = (msta & STATUS_HOMED)     > 0 ? 1 : 0;
  gain_supp = (msta & STATUS_GAIN_SUPP) > 0 ? 1 : 0;
  driveOn   = (msta & STATUS_GAIN_SUPP) > 0 ? 1 : 0;
  mov       = (msta & STATUS_MOVING)    > 0 ? 1 : 0;

  // Send the MotorOff command if this motor is set to auto_off and moving == 0
  if(axisAutoOff_ == 1 && mov == 0)
  {
    if(autoOffRetries_ == 10)
      setClosedLoop(0);
    autoOffRetries_++;
  }
  else if (mov == 1)
    autoOffRetries_ = 0;

  // Read the drive power on status
  setIntegerParam(pC_->motorStatusAtHome_, limit);
  setIntegerParam(pC_->motorStatusDone_, done);
  setIntegerParam(pC_->motorStatusHomed_, homed);
  setIntegerParam(pC_->motorStatusGainSupport_, gain_supp);
  setIntegerParam(pC_->motorStatusPowerOn_, driveOn);
  setIntegerParam(pC_->motorStatusMoving_, mov);
  setIntegerParam(pC_->motorStatusProblem_, 0);

  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg HexapodCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg HexapodCreateControllerArg1 = {"Hexapod port name", iocshArgString};
static const iocshArg HexapodCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg HexapodCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg HexapodCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const HexapodCreateControllerArgs[] = {&HexapodCreateControllerArg0,
                                                           &HexapodCreateControllerArg1,
                                                           &HexapodCreateControllerArg2,
                                                           &HexapodCreateControllerArg3,
                                                           &HexapodCreateControllerArg4};
static const iocshFuncDef HexapodCreateControllerDef = {"HexapodCreateController", 5, HexapodCreateControllerArgs};

// Motor Name Config arguments
static const iocshArg motorArg0 = {"Motor number", iocshArgInt};
static const iocshArg motorArg1 = {"Motor name", iocshArgString};
static const iocshArg * const MotorNameConfigArgs[2]  = {&motorArg0, &motorArg1};
static const iocshFuncDef configMotorName = {"MotorNameConfig", 2, MotorNameConfigArgs};

// Motor auto_off config
static const iocshArg motorAutoOffArg0 = {"Motor number", iocshArgInt};
static const iocshArg motorAutoOffArg1 = {"Auto off", iocshArgInt};
static const iocshArg * const MotorAutoOffConfigArgs[2]  = {&motorAutoOffArg0, &motorAutoOffArg1};
static const iocshFuncDef configMotorAutoOff = {"MotorAutoOffConfig", 2, MotorAutoOffConfigArgs};

static void HexapodCreateContollerCallFunc(const iocshArgBuf *args)
{
  HexapodCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void configMotorNameCallFunc(const iocshArgBuf *args)
{
  MotorNameConfig(args[0].ival, args[1].sval);
}

static void configMotorAutoOffCallFunc(const iocshArgBuf *args)
{
  MotorAutoOffConfig(args[0].ival, args[1].ival);
}

static void HexapodMotorRegister(void)
{
  iocshRegister(&HexapodCreateControllerDef, HexapodCreateContollerCallFunc);
  iocshRegister(&configMotorName, configMotorNameCallFunc);
  iocshRegister(&configMotorAutoOff, configMotorAutoOffCallFunc);
}

extern "C" {
epicsExportRegistrar(HexapodMotorRegister);
}
