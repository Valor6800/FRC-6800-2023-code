#include <iostream>
#include "subsystems/Intake.h"



Intake::Intake(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "Intake"),  
                            intakeMotor(10, rev::CANSparkMax::IdleMode::kCoast, false)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Intake::~Intake()
{
    
}


void Intake::resetState()
{
   
}

void Intake::init()
{
    // table->PutNumber("Motor 1 Max Speed", motor1MaxSpeed);
    // table->PutNumber("Motor 2 Max Speed", motor2MaxSpeed);
    // table->PutNumber("Motor 3 Max Speed", 0.0); 

    resetState();
}

void Intake::assessInputs()
{
    
    // state.leftStickSpeed = driverGamepad->leftStickY(2);
    // state.rightStickSpeed = driverGamepad->rightStickY(2);
}
void Intake::analyzeDashboard()
{
    // motor1MaxSpeed = table->GetNumber("Motor 1 Max Speed", 0);
    // motor2MaxSpeed = table->GetNumber("Motor 2 Max Speed", 0);
    // table->GetNumber("Motor 3 Max Speed", motor3MaxSpeed);

}

void Intake::assignOutputs()
{    
    // testMotor1.setPower(state.leftStickSpeed * motor1MaxSpeed);
    // testMotor2.setPower(state.rightStickSpeed * motor2MaxSpeed);
}