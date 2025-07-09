#include <fstream>
#include "Robot.h"
#include "AMCU.h"
#include "Constants.h"
#include "Utilities/log.hpp"
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

AMCU amcu;

bool testWasEnabled = false; 

void Robot::RobotInit() {
    amcu.initOmniDriveBase(kWheelRadius, kRobotRadius, kMotorLeft, kMotorRight, kMotorBack);
}

void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {
    LOG_DISABLED(" Disabled");
    testWasEnabled = false;
}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {
    testWasEnabled = false;
    LOG_AUTONOMOUS("Enabled");
    last_mode = {PURPLE, "[AUTONOMOUS]"};
    //amcu.driveDistance(2, 0, 0); // Example: drive 2 meters in x
}

void Robot::AutonomousPeriodic() {
    frc::SmartDashboard::PutNumber("Encoder Left", amcu.getEncoder(kMotorLeft));
    frc::SmartDashboard::PutNumber("Encoder Right", amcu.getEncoder(kMotorRight));
    frc::SmartDashboard::PutNumber("Encoder Back", amcu.getEncoder(kMotorBack));
}

void Robot::TeleopInit() {
    testWasEnabled = false;
    LOG_TELEOP("Enabled");
    last_mode = {CYAN, "[TELEOP]"};
    // If you use command-based, make sure to stop auto commands here
    // if (m_autonomousCommand != nullptr) {
    //   m_autonomousCommand->Cancel();
    //   m_autonomousCommand = nullptr;
    // }
}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {
    bool currentlyEnabled = IsEnabled();
    if (currentlyEnabled && !testWasEnabled) {
        last_mode = {YELLOW, "[TEST]"};
        LOG_TEST("Enabled");
    }
    testWasEnabled = currentlyEnabled;

}

#ifndef RUNNING_FRC_TESTS
int main() {
    SetupLogging();
    return frc::StartRobot<Robot>();
}
#endif