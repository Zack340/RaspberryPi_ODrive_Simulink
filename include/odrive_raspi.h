/* 
 *  	Author : Eisuke Matsuzaki
 *  	Created on : 08/03/2020
 *  	Copyright (c) 2020 d’Arbeloff Lab, MIT Department of Mechanical Engineering
 *      Released under the GNU license
 * 
 *      ODrive driver for Raspberry Pi
 */ 

#ifndef _ODRIVE_RASPI_H_
#define _ODRIVE_RASPI_H_
#include "rtwtypes.h"
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h> 
#include <termios.h>
#include <sys/stat.h>
#include <dirent.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

enum State{
    AXIS_STATE_UNDEFINED = 0,
    AXIS_STATE_IDLE,
    AXIS_STATE_STARTUP_SEQUENCE,
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_MOTOR_CALIBRATION,
    AXIS_STATE_SENSORLESS_CONTROL, 
    AXIS_STATE_ENCODER_INDEX_SEARCH,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_LOCKIN_SPIN,
    AXIS_STATE_ENCODER_DIR_FIND,
};

enum ControlMode{
    CTRL_MODE_VOLTAGE_CONTROL = 0,
    CTRL_MODE_CURRENT_CONTROL,
    CTRL_MODE_VELOCITY_CONTROL,
    CTRL_MODE_POSITION_CONTROL,
    CTRL_MODE_TRAJECTORY_CONTROL
};

struct Settings
{
    boolean_T isPort;
    uint8_T serial[64];
    uint8_T portName[64];
    boolean_T isExternal[2];
    boolean_T isAxis[2];
    uint16_T controlMode[2];
    real32_T posGain[2];
    real32_T velGain[2];
    real32_T velIntegratorGain[2];
    real32_T velLimit[2];
    real32_T velLimitTolerance[2];
    real32_T velRampRate[2];
    boolean_T setPointsInCpr[2];
    boolean_T velRampEnable[2];
    real32_T watchdogTimeout[2];
};

struct Data
{
    int16_T error[2];
    real32_T posSetpoint[2];
    real32_T velSetpoint[2];
    real32_T currentSetpoint[2];
    real32_T posGain[2];
    real32_T velGain[2];
    real32_T velIntegratorGain[2];
    real32_T velIntegratorCurrentRef[2];
    boolean_T velIntegratorCurrentTrigger[2];
    boolean_T velRampEnable[2];
    real32_T velRampTarget[2];
    real32_T velRampRate[2];
    real32_T velLimit[2];
    real32_T velLimitTolerance[2];
    real32_T actualPosition[2];
    real32_T actualVelocity[2];
    real32_T actualCurrent[2];
    real32_T velIntegratorCurrentAct[2];
};

void initialize(struct Settings *settings);
void step(struct Data *data);
void terminate();
uint8_T detectOdrivePort(uint8_T *serial, uint8_T *portName);
int32_T openSerialPort(uint8_T *portName);
void startupSequence(int32_T f, uint8_T axis);
void waitSetupStatus(int32_T f, struct Settings *settings);
void setConfiguration(int32_T f, uint8_T axis, struct Settings *settings);
void sendMessage(int32_T f, uint8_T *message);
void receiveMessage(int32_T f, uint8_T *message, uint8_T len);

#ifdef __cplusplus
}
#endif
#endif //_ODRIVE_RASPI_H_