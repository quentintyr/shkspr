/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ExampleSubsystem.h"
#include "subsystems/AMCU.h"
#include <utilities/LoggingSystem.h>
#include <utilities/Constants.h>
#include <frc/smartdashboard/SmartDashboard.h>

class DriveController
{
private:
  /* data */
public:
  DriveController(/* args */);
  ~DriveController();
};

DriveController::DriveController(/* args */)
{
}

DriveController::~DriveController()
{
}
 void DriveController() {


}

void DisplayData() {
  LOG_INFO("Displaying in Smart Dashboard");
  frc::SmartDashboard::PutNumber("Encoder Left", amcu.getEncoder(kMotorLeft));
  frc::SmartDashboard::PutNumber("Encoder Right", amcu.getEncoder(kMotorRight));
  frc::SmartDashboard::PutNumber("Encoder Back", amcu.getEncoder(kMotorBack));

}
