#include <subsystems/LimitSwitchSubsystem.h>

LimitSwitchSubsystem::LimitSwitchSubsystem(int port) : m_limitSwitch(port) {

}

bool LimitSwitchSubsystem::IsPressed() const {
    return m_limitSwitch.Get();
}

std::string LimitSwitchSubsystem::GetStateString() const {
    return "Switch State: " + std::string(IsPressed() ? "Pressed" : "Closed"); // Electricity is running
}