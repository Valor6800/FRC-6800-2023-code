#include <iostream>
#include "subsystems/Intake.h"
//#include"subsystems/Elevarm.h


Intake::Intake(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "Intake"),  
                            intakeMotor(10, rev::CANSparkMax::IdleMode::kCoast, false),
                            currySensor(_robot),
                            intakeSpeed(-1),
                            outtakeSpeed(1),
                            spikeAmps(50)
{  
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Intake::~Intake()
{
    
}


void Intake::resetState()
{
   state.intakeState = DISABLED;
   currySensor.reset();
}

void Intake::init()
{
    
    intakeMotor.setupFollower(11,true);
    //table->PutNumber("Motor 1 Max Speed", motor1MaxSpeed);
    // table->PutNumber("Motor 2 Max Speed", motor2MaxSpeed);
    // table->PutNumber("Motor 3 Max Speed", 0.0); 
    currySensor.setSpikeSetpoint(spikeAmps);
    currySensor.setGetter([this]() { return intakeMotor.getCurrent(); });
    currySensor.setSpikeCallback([this]() { state.intakeState = SPIKED;});
    table->PutNumber("Intake Speed",intakeSpeed);
    table->PutNumber("Outtake Speed",outtakeSpeed);
    table->PutNumber("Spiked Amperage",spikeAmps);
    resetState();
}

void Intake::assessInputs()
{
    if (driverGamepad->GetAButton()){
            // works with elevarm locations in order to outtake at speed for cone/cube
            if (operatorGamepad->DPadDown() || operatorGamepad->DPadUp()
                || operatorGamepad->DPadLeft() || operatorGamepad->DPadRight()){
                state.intakeState = OUTTAKE_CUBE;
            } else if (operatorGamepad->GetAButton() || operatorGamepad->GetBButton() 
               || operatorGamepad->GetXButton() || operatorGamepad->GetYButton()){
                state.intakeState = OUTTAKE_CONE;
            } else {
                state.intakeState = OUTTAKE;
            }
    } else if (operatorGamepad->rightTriggerActive()){
        state.intakeState = OUTTAKE;
    }else if (state.intakeState != SPIKED){
        //need to add current sensing abilities here?
        if (driverGamepad->GetLeftBumper() || driverGamepad->GetRightBumper() || 
        operatorGamepad->leftTriggerActive()){
        //move elevarm to front position
        state.intakeState = INTAKE;
        } else{
            state.intakeState = DISABLED;
        }
    } else if(state.intakeState == SPIKED){

        if (!driverGamepad->GetLeftBumper() && !driverGamepad->GetRightBumper() && 
        !operatorGamepad->leftTriggerActive()){
        //move elevarm to front position
        state.intakeState = DISABLED;
        } 
    }
    //need to check when button is released in order to reset Spiked
    //might have to add a 
    // state.leftStickSpeed = driverGamepad->leftStickY(2);
    // state.rightStickSpeed = driverGamepad->rightStickY(2);
}
void Intake::analyzeDashboard()
{
    table->PutNumber("Intake Current", currySensor.getSensor());
    table->PutNumber("State", state.intakeState);
    intakeSpeed = table->GetNumber("Intake Speed",0);
    outtakeSpeed = table->GetNumber("Outtake Speed",0);
    spikeAmps = table->GetNumber("Spiked Amperage",0);
    // motor1MaxSpeed = table->GetNumber("Motor 1 Max Speed", 0);
    // motor2MaxSpeed = table->GetNumber("Motor 2 Max Speed", 0);
    // table->GetNumber("Motor 3 Max Speed", motor3MaxSpeed);

}
 
void Intake::assignOutputs()
{ 

    if (state.intakeState == DISABLED || state.intakeState == SPIKED){
        intakeMotor.setPower(0);
    } else if (state.intakeState == OUTTAKE_CONE){
        intakeMotor.setPower(-0.5);
    } else if (state.intakeState == OUTTAKE_CUBE){
        intakeMotor.setPower(-0.25);
    } else if (state.intakeState == OUTTAKE){
        intakeMotor.setPower(outtakeSpeed);
    } else if (state.intakeState == INTAKE){
        intakeMotor.setPower(intakeSpeed);
    }
    //add current sensing logic here    
    // testMotor1.setPower(state.leftStickSpeed * motor1MaxSpeed);
    // testMotor2.setPower(state.rightStickSpeed * motor2MaxSpeed);
}