#include <fstream>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

#include "Robot.h"
#include "subsystems/AMCU.h"
#include "subsystems/LimitSwitchSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "utilities/Constants.h"
#include "utilities/LoggingSystem.h"

bool testWasEnabled = false;
bool firstTimeStartUP = false;



std::unique_ptr<RobotContainer> robotContainer;
LimitSwitchSubsystem limitSwitch(kElevatorUpLimitSwitch);

void Robot::RobotInit()
{
    robotContainer = std::make_unique<RobotContainer>();
}

void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit()
{
    
    if (!firstTimeStartUP)
    {
        firstTimeStartUP = true;
        LOG_INFO("First Time Startup");
    }
    else
    {
        LOG_DISABLED(" Disabled");
        testWasEnabled = false;
    }
}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit()
{
    testWasEnabled = false;
    LOG_AUTONOMOUS("Enabled");
    last_mode = {PURPLE, "[AUTONOMOUS]"};

    // omnidrive train test
    // amcu.driveDistance(1, 0, 0); // Example: drive 2 meters in x

    // Elevator Movement

    // amcu.setSpeed(MOTOR_0, 10); // 5% speed
    // LOG_INFO("M0 5% Speed for 1 second")
    // std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait 1s
    // amcu.setSpeed(MOTOR_0, 0);
}

void Robot::AutonomousPeriodic()
{
}

void Robot::TeleopInit()
{
    testWasEnabled = false;
    LOG_TELEOP("Enabled");
    last_mode = {CYAN, "[TELEOP]"};

    myServo.SetAngle(150);
}

void Robot::TeleopPeriodic()
{

}

void Robot::TestPeriodic()
{

}

#ifndef RUNNING_FRC_TESTS
int main()
{
    SetupLogging();
    return frc::StartRobot<Robot>();
}
#endif