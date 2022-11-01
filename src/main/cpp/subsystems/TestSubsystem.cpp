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
#include "frc/smartdashboard/SmartDashboard.h"

#define TEST_CONVERSION_FACTOR 1.0 * 360

#define TEST_CAN_ID 10
#define TEST_FOLLOWER_CAN_ID 9

#define LIMIT_SWITCH_DIO_PORT 0

TestSubsystem::TestSubsystem(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "TestSubsystem"),
                           operatorController(NULL),
                           testMotorController(TEST_CAN_ID, rev::CANSparkMax::IdleMode::kCoast, false),
                           limitOne(_robot),
                           limitSwitch(LIMIT_SWITCH_DIO_PORT)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}


void TestSubsystem::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("double");
    builder.AddDoubleProperty(
        "value",
        [this] { return state.testSubsystemState; },
        nullptr
    );
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

    table->PutNumber("Test Position Target", 5);
    table->PutNumber("Test Speed Target", 2);
    table->PutNumber("Test Power Target", 0.5);

    limitOne.setGetter([this]() { return limitSwitch.Get(); });
    
    limitOne.setRisingEdgeCallback([this]() {
        state.testSubsystemState = TestSubsystemState::TOPOWER;
    });

    limitOne.setFallingEdgeCallback([this]() {
        state.testSubsystemState = TestSubsystemState::DISABLED;
    });
    
    resetState();
}

void TestSubsystem::setControllers(ValorGamepad *controllerO)
{
    operatorController = controllerO;
}

void TestSubsystem::assessInputs()
{
    if (!operatorController)
    {
        return;
    }
        
    // if (operatorController->GetAButton()) {
    //     state.testSubsystemState = TestSubsystemState::TOPOWER;
    // }
    // else if (operatorController->GetBButton()) {
    //     state.testSubsystemState = TestSubsystemState::TOPOSITION;
    // }
    // else if (operatorController->GetXButton()) {
    //     state.testSubsystemState = TestSubsystemState::TOSPEED;
    // }
    // else {
    //     state.testSubsystemState = TestSubsystemState::DISABLED;
    // }
}

void TestSubsystem::analyzeDashboard()
{
    table->PutNumber("Test Motor Position", testMotorController.getPosition());
    table->PutNumber("Test Motor Speed", testMotorController.getSpeed());
    table->PutNumber("Test Motor Current", testMotorController.getCurrent());

    table->PutNumber("Test Subsystem State", state.testSubsystemState);
    
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
        testMotorController.setPosition(state.testPositionTarget);
    }
    else if (state.testSubsystemState == TestSubsystemState::TOSPEED) {
        testMotorController.setSpeed(state.testSpeedTarget);
    }
}

void TestSubsystem::resetState()
{
    state.testSubsystemState = TestSubsystem::DISABLED;
}