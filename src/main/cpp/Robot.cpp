#include "Robot.h"
#include "AMCU.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

constexpr int kWheelRadius = 50;
constexpr int kRobotRadius = 150;
constexpr Motor kMotorLeft  = MOTOR_1;
constexpr Motor kMotorRight = MOTOR_3;
constexpr Motor kMotorBack  = MOTOR_0;

AMCU amcu;

void Robot::RobotInit() {
  amcu.initOmniDriveBase(kWheelRadius, kRobotRadius, kMotorLeft, kMotorRight, kMotorBack);
}

void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {
  //amcu.driveDistance(2, 0, 0); // Example: drive 2 meters in x
}

void Robot::AutonomousPeriodic() {
  wpi::outs() << "example\n";
  frc::SmartDashboard::PutNumber("Encoder Left", amcu.getEncoder(kMotorLeft));
  frc::SmartDashboard::PutNumber("Encoder Right", amcu.getEncoder(kMotorRight));
  frc::SmartDashboard::PutNumber("Encoder Back", amcu.getEncoder(kMotorBack));
}

void Robot::TeleopInit() {
  // If you use command-based, make sure to stop auto commands here
  // if (m_autonomousCommand != nullptr) {
  //   m_autonomousCommand->Cancel();
  //   m_autonomousCommand = nullptr;
  // }
}

void Robot::TeleopPeriodic() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif