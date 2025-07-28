#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <string>

class LimitSwitchSubsystem : public frc2::SubsystemBase {
public:
    explicit LimitSwitchSubsystem(int port); // no type conversion

    bool IsPressed() const;
    std::string GetStateString() const;

private:
    frc::DigitalInput m_limitSwitch;
};