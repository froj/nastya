/**
 @file cvra_cs.c
 @author Antoine Albertelli
 @date 19th September 2009
 @brief This files implements the control system loop wrappers.
 
 The file provides all the implementation of the Control System Management (CSM).
 A CSM is made of 3 parts : acceleration / deceleration ramp generator 
 (Quadramp), a position regulator (PID), and I/O interfaces (PWM & quadratures
 encoders).
 The quadramp tells the PID what position the wheel should be, depending on the 
 acceleration and time. This value is then fed as the consign value to the PID 
 regulator, with the encoder value as the measured position. The output of the 
 PID is then applied to the motor via the PWM.
 The others functions computed here are the position manager, the trajectory
 manager and the blocking detection system.
 */

#include <aversive.h>

#include <2wheels/trajectory_manager.h>
#include <2wheels/robot_system.h>
#include <2wheels/position_manager.h>
#include <control_system_manager.h>
#include <pid.h>
#include <quadramp.h>
#include <scheduler.h>

#include <aversive/error.h>
#include "error_numbers.h"
#include "adresses.h"

#include <string.h>
#include <stdio.h>

#include "cvra_cs.h"
#include "hardware.h"
#include <cvra_bldc.h>
#include "cvra_param_robot.h"


struct _rob robot;


void cvra_cs_init(void) {

}

/**
 @brief Brings power back on after a blocking.
 
 This functions resets the blocking detection systems, sets the robot on angle
 and distance mode and stops the current trajectory.
 */
static void restart_power(__attribute__((unused)) void * dummy) {

}

/** Logge l'erreur sur les differents regulateurs et l'affiche avec le temps. */
static void dump_error(void) {

}

void cvra_cs_manage(__attribute__((unused)) void * dummy) {

}
