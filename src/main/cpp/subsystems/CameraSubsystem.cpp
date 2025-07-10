#include "subsystems/CameraSubsystem.h"
#include <cameraserver/CameraServer.h>

CameraSubsystem::CameraSubsystem(){
    frc::CameraServer::GetInstance()->StartAutomaticCapture();

};