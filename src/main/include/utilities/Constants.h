/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <subsystems/AMCU.h>

#pragma once

extern AMCU amcu;

// amcu omni drive base variables
constexpr int kWheelRadius = 50;
constexpr int kRobotRadius = 150;
constexpr Motor kMotorLeft  = MOTOR_1;
constexpr Motor kMotorRight = MOTOR_3;
constexpr Motor kMotorBack  = MOTOR_0;

const int MAP_length = 400; //cm
const int MAP_width = 200;



/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
