#include <iostream>
#include "subsystems/Intake.h"
//#include"subsystems/Elevarm.h


Intake::Intake(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "Intake"),  
                            intakeMotor(10, rev::CANSparkMax::IdleMode::kCoast, false),
                            currySensor(_robot)
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
    intakeMotor.setupFollower(11,true);
    //table->PutNumber("Motor 1 Max Speed", motor1MaxSpeed);
    // table->PutNumber("Motor 2 Max Speed", motor2MaxSpeed);
    // table->PutNumber("Motor 3 Max Speed", 0.0); 
    currySensor.setSpikeSetpoint(20);
    currySensor.setGetter([this]() { return intakeMotor.getCurrent(); });
    currySensor.setSpikeCallback([this]() { state.intakeState = SPIKED;});
    resetState();
}

void Intake::assessInputs()
{
    if (driverGamepad->GetAButtonPressed()){
            // works with elevarm locations in order to outtake at speed for cone/cube
            if (ELEVARM_CONE){
                state.intakeState = OUTTAKE_CONE;
            } else if (ELEVARM_CUBE){
                state.intakeState = OUTTAKE_CUBE;
            }
    } else if (operatorGamepad->rightTriggerActive()){
        state.intakeState = OUTTAKE;
    }else if (state.intakeState != SPIKED){
        //need to add current sensing abilities here?
        if (driverGamepad->GetLeftBumperPressed() || driverGamepad->GetRightBumperPressed() || 
        operatorGamepad->leftTriggerActive()){
        //move elevarm to front position
        state.intakeState = INTAKE;
        }
        
    } else if(state.intakeState == SPIKED){

        if (!driverGamepad->GetLeftBumperPressed() && !driverGamepad->GetRightBumperPressed() && 
        !operatorGamepad->leftTriggerActive()){
        //move elevarm to front position
        state.intakeState = DISABLED;
        } 
    }
     else {

        state.intakeState = DISABLED;
    }
    //need to check when button is released in order to reset Spiked
    //might have to add a 
    // state.leftStickSpeed = driverGamepad->leftStickY(2);
    // state.rightStickSpeed = driverGamepad->rightStickY(2);
}
void Intake::analyzeDashboard()
{
    table->PutNumber("Intake Current", currySensor.getSensor());
    // motor1MaxSpeed = table->GetNumber("Motor 1 Max Speed", 0);
    // motor2MaxSpeed = table->GetNumber("Motor 2 Max Speed", 0);
    // table->GetNumber("Motor 3 Max Speed", motor3MaxSpeed);

}

void Intake::assignOutputs()
{ 
    if (state.intakeState == DISABLED || state.intakeState == SPIKED){
        intakeMotor.setSpeed(0);
    } else if (state.intakeState == OUTTAKE_CONE){
        intakeMotor.setSpeed(-0.5);
    } else if (state.intakeState == OUTTAKE_CUBE){
        intakeMotor.setSpeed(-0.25);
    } else if (state.intakeState == INTAKE){
        intakeMotor.setSpeed(0.5);
    } else {
        intakeMotor.setSpeed(0);
    }
    //add current sensing logic here    
    // testMotor1.setPower(state.leftStickSpeed * motor1MaxSpeed);
    // testMotor2.setPower(state.rightStickSpeed * motor2MaxSpeed);
}