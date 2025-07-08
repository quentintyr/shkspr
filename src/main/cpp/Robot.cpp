#include <fstream>
#include "Robot.h"
#include "AMCU.h"
#include "Constants.h"
#include "Utilities/log.hpp" // rio log does not work idk why
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

AMCU amcu;

void Robot::RobotInit() {
  amcu.initOmniDriveBase(kWheelRadius, kRobotRadius, kMotorLeft, kMotorRight, kMotorBack);
}

void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

void Robot::DisabledInit() {
  LOG_DISABLED("Disabled");
}
void Robot::DisabledPeriodic() {
}

void Robot::AutonomousInit() {
  LOG_AUTONOMOUS("Enabled");
  //amcu.driveDistance(2, 0, 0); // Example: drive 2 meters in x
  // std::ofstream logFile("home/pi/robot.log"), std::ios::out | std::ios::app;
  // logFile << "This is a log message." << std::endl;
}

void Robot::AutonomousPeriodic() {
  frc::SmartDashboard::PutNumber("Encoder Left", amcu.getEncoder(kMotorLeft));
  frc::SmartDashboard::PutNumber("Encoder Right", amcu.getEncoder(kMotorRight));
  frc::SmartDashboard::PutNumber("Encoder Back", amcu.getEncoder(kMotorBack));
}

void Robot::TeleopInit() {
  LOG_TELEOP("Enabled");
  // If you use command-based, make sure to stop auto commands here
  // if (m_autonomousCommand != nullptr) {
  //   m_autonomousCommand->Cancel();
  //   m_autonomousCommand = nullptr;
  // }
}

void Robot::TeleopPeriodic() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { 
  SetupLogging();
  return frc::StartRobot<Robot>(); 
  }
#endif