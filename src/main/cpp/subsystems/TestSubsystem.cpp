/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//start button pulls swerve 0 positions from file
//back button pushes current swerve positions to file


#include "subsystems/TestSubsystem.h"
#include <iostream>

#define TEST_CONVERSION_FACTOR 1.0
#define TEST_CAN_ID 5
#define TEST_FOLLOWER_CAN_ID 6

TestSubsystem::TestSubsystem(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "TestSubsystem"),
                           driverController(NULL),
                           operatorController(NULL),
                           testMotorController(TEST_CAN_ID, Coast, false)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void TestSubsystem::init()
{

    testMotorController.setConversion(TEST_CONVERSION_FACTOR);

    //testMotorController.setupFollower(TEST_FOLLOWER_CAN_ID);
    
    //testMotorController.setPIDF(int slot, PIDF pidf);
    //testMotorController.setLimits(int reverse, int forward);
    //testMotorController.setForwardLimit(int forward);
    //testMotorController.setReverseLimit(int reverse);
    //testMotorController.setRange(int slot, double min, double max);
    
    //testMotorController.setProfile(int slot);

    table->PutNumber("Test Position Target", 0);
    table->PutNumber("Test Speed Target", 0);
    table->PutNumber("Test Power Target", 0);
    
    resetState();
}

void TestSubsystem::setControllers(ValorGamepad *controllerO, ValorGamepad *controllerD)
{
    driverController = controllerD;
    operatorController = controllerO;
}

void TestSubsystem::assessInputs()
{
    if (!driverController)
    {
        return;
    }
        
    if (operatorController->GetAButton()) {
        state.testSubsystemState = TestSubsystemState::TOPOWER;
    }
    else if (operatorController->GetBButton()) {
        state.testSubsystemState = TestSubsystemState::TOPOSITION;
    }
    else if (operatorController->GetXButton()) {
        state.testSubsystemState = TestSubsystemState::TOSPEED;
    }
    else {
        state.testSubsystemState = TestSubsystemState::DISABLED;
    }
}

void TestSubsystem::analyzeDashboard()
{
    table->PutNumber("Test Motor Position", testMotorController.getPosition());
    table->PutNumber("Test Motor Speed", testMotorController.getSpeed());
    table->PutNumber("Test Motor Current", testMotorController.getCurrent());
    
    state.testPositionTarget = table->GetNumber("Test Position Target", 0);
    state.testPowerTarget = table->GetNumber("Test Power Target", 0);
    state.testSpeedTarget = table->GetNumber("Test Speed Target", 0);
}

void TestSubsystem::assignOutputs()
{

    if (state.testSubsystemState == TestSubsystemState::DISABLED) {
        testMotorController.setPower(0);
    }
    else if (state.testSubsystemState == TestSubsystemState::TOPOWER) {
        testMotorController.setPower(state.testPowerTarget);
    }
    else if (state.testSubsystemState == TestSubsystemState::TOPOSITION) {
        testMotorController.setPower(state.testPositionTarget);
    }
    else if (state.testSubsystemState == TestSubsystemState::TOSPEED) {
        testMotorController.setPower(state.testSpeedTarget);
    }
}

void TestSubsystem::resetState()
{
    state.testSubsystemState = TestSubsystem::DISABLED;
}