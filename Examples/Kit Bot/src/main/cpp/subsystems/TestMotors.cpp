/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/TestMotors.h"
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/shuffleboard/Shuffleboard.h>

TestMotors::TestMotors() : ValorSubsystem(),
                           m_testMotor1{TestMotorsConstants::TEST_MOTOR_CAN_ID_1 , rev::CANSparkMax::MotorType::kBrushless},
                           m_testMotor2{TestMotorsConstants::TEST_MOTOR_CAN_ID_2 , rev::CANSparkMax::MotorType::kBrushless},
                           m_testMotor3{TestMotorsConstants::TEST_MOTOR_CAN_ID_3 , rev::CANSparkMax::MotorType::kBrushless},
                           m_testMotor4{TestMotorsConstants::TEST_MOTOR_CAN_ID_4 , rev::CANSparkMax::MotorType::kBrushless},
                           m_testMotor5{TestMotorsConstants::TEST_MOTOR_CAN_ID_5 , rev::CANSparkMax::MotorType::kBrushless},
                           m_testMotor6{TestMotorsConstants::TEST_MOTOR_CAN_ID_6 , rev::CANSparkMax::MotorType::kBrushless},
                           m_testMotor7{TestMotorsConstants::TEST_MOTOR_CAN_ID_7 , rev::CANSparkMax::MotorType::kBrushless},
                           m_testMotor8{TestMotorsConstants::TEST_MOTOR_CAN_ID_8 , rev::CANSparkMax::MotorType::kBrushless},
                           m_testMotor9{TestMotorsConstants::TEST_MOTOR_CAN_ID_9 , rev::CANSparkMax::MotorType::kBrushless},
                           m_testMotor10{TestMotorsConstants::TEST_MOTOR_CAN_ID_10 , rev::CANSparkMax::MotorType::kBrushless},
                           driverController(NULL),
                           operatorController(NULL),
                           testPhotoelec(TestMotorsConstants::TEST_PHOTO_ELEC_DIO_PORT)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

TestMotors::~TestMotors()
{

}


void TestMotors::init()
{
    limeTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    initTable("TestMotors");

    state.motors[0] = &m_testMotor1;
    state.motors[1] = &m_testMotor2;
    state.motors[2] = &m_testMotor3;
    state.motors[3] = &m_testMotor4;
    state.motors[4] = &m_testMotor5;
    state.motors[5] = &m_testMotor6;
    state.motors[6] = &m_testMotor7;
    state.motors[7] = &m_testMotor8;
    state.motors[8] = &m_testMotor9;
    state.motors[9] = &m_testMotor10;

    m_testMotor4.Follow(m_testMotor2, false);

    state.controllers[0] = "Driver";
    state.controllers[1] = "Operator";

    state.buttons[0] = "A Button";
    state.buttons[1] = "B Button";
    state.buttons[2] = "X Button";
    state.buttons[3] = "Y Button";
    state.buttons[4] = "DPad Up";
    state.buttons[5] = "DPad Down";
    state.buttons[6] = "DPad Right";
    state.buttons[7] = "DPad Left";
    state.buttons[8] = "Start Button";
    state.buttons[9] = "Back Button";
    state.buttons[10] = "Left Stick Y";
    state.buttons[11] = "Left Stick X";
    state.buttons[12] = "Right Stick Y";
    state.buttons[13] = "Right Stick X";
    state.buttons[14] = "Left Trigger";
    state.buttons[15] = "Right Trigger";
    state.buttons[16] = "Left Bumper";
    state.buttons[17] = "Right Bumper";

    
    table->PutNumber("Test Motor ID 5", 0.0);  //intake
    table->PutNumber("Test Motor ID 6", 0.0); //top
    table->PutNumber("Test Motor ID 7", 0.0);
    //table->PutNumber("Test Motor ID 8", 0.0);  //bottom
    table->PutNumber("Test Motor ID 9", 0.0);
    table->PutNumber("Test Motor ID 10", 0.0);  //feeder
    table->PutNumber("Test Motor ID 11", 0.0);
    table->PutNumber("Test Motor ID 12", 0.0);
    table->PutNumber("Test Motor ID 13", 0.0);
    table->PutNumber("Test Motor ID 14", 0.0);
}

void TestMotors::setControllers(frc::XboxController *controllerO, frc::XboxController *controllerD)
{
    driverController = controllerD;
    operatorController = controllerO;
}

void TestMotors::assessInputs()
{
    if (!driverController)
    {
        return;
    }

    // driver inputs
    state.driver_leftStickX = driverController->GetLeftX();
    state.driver_leftStickY = driverController->GetLeftY();
    state.driver_rightStickX = driverController->GetRightX();
    state.driver_rightStickY = driverController->GetRightY();

    state.driver_bButtonPressed = driverController->GetBButton();
    state.driver_aButtonPressed = driverController->GetAButton();
    state.driver_xButtonPressed = driverController->GetXButton();
    state.driver_yButtonPressed = driverController->GetYButton();

    driverController->GetPOV() == OIConstants::dpadDown ? state.driver_dPadDownPressed = true : state.driver_dPadDownPressed = false;
    driverController->GetPOV() == OIConstants::dpadUp ? state.driver_dPadUpPressed = true : state.driver_dPadUpPressed = false;
    driverController->GetPOV() == OIConstants::dpadRight ? state.driver_dPadRightPressed = true : state.driver_dPadRightPressed = false;
    driverController->GetPOV() == OIConstants::dpadLeft ? state.driver_dPadLeftPressed = true : state.driver_dPadLeftPressed = false;

    driverController->GetLeftTriggerAxis() > OIConstants::kTriggerDeadband ? state.driver_leftTriggerPressed = true : state.driver_leftTriggerPressed = false;
    driverController->GetRightTriggerAxis() > OIConstants::kTriggerDeadband ? state.driver_rightTriggerPressed = true : state.driver_rightTriggerPressed = false;

    state.driver_startButtonPressed = driverController->GetStartButtonPressed();
    state.driver_backButtonPressed = driverController->GetBackButtonPressed(); 

    state.driver_leftBumperPressed = driverController->GetLeftBumperPressed();
    state.driver_rightBumperPressed = driverController->GetRightBumperPressed();



    // operator inputs
    state.operator_leftStickX = operatorController->GetLeftX();
    state.operator_leftStickY = operatorController->GetLeftY();
    state.operator_rightStickX = operatorController->GetRightX();
    state.operator_rightStickY = operatorController->GetRightY();

    state.operator_bButtonPressed = operatorController->GetBButton();
    state.operator_aButtonPressed = operatorController->GetAButton();
    state.operator_xButtonPressed = operatorController->GetXButton();
    state.operator_yButtonPressed = operatorController->GetYButton();

    operatorController->GetPOV() == OIConstants::dpadDown ? state.operator_dPadDownPressed = true : state.operator_dPadDownPressed = false;
    operatorController->GetPOV() == OIConstants::dpadUp ? state.operator_dPadUpPressed = true : state.operator_dPadUpPressed = false;
    operatorController->GetPOV() == OIConstants::dpadRight ? state.operator_dPadRightPressed = true : state.operator_dPadRightPressed = false;
    operatorController->GetPOV() == OIConstants::dpadLeft ? state.operator_dPadLeftPressed = true : state.operator_dPadLeftPressed = false;

    operatorController->GetLeftTriggerAxis() > OIConstants::kTriggerDeadband ? state.operator_leftTriggerPressed = true : state.operator_leftTriggerPressed = false;
    operatorController->GetRightTriggerAxis() > OIConstants::kTriggerDeadband ? state.operator_rightTriggerPressed = true : state.operator_rightTriggerPressed = false;

    state.operator_startButtonPressed = operatorController->GetStartButtonPressed();
    state.operator_backButtonPressed = operatorController->GetBackButtonPressed(); 

    state.operator_leftBumperPressed = operatorController->GetLeftBumperPressed();
    state.operator_rightBumperPressed = operatorController->GetRightBumperPressed();

    state.photoelecReading = testPhotoelec.Get();
}

void TestMotors::analyzeDashboard()
{
    state.testMotorVal[0] = table->GetNumber("Test Motor ID 5", 0.0);
    state.testMotorVal[1] = table->GetNumber("Test Motor ID 6", 0.0);
    state.testMotorVal[2] = table->GetNumber("Test Motor ID 7", 0.0);
    //state.testMotorVal[3] = table->GetNumber("Test Motor ID 8", 0.0);
    state.testMotorVal[4] = table->GetNumber("Test Motor ID 9", 0.0);
    state.testMotorVal[5] = table->GetNumber("Test Motor ID 10", 0.0);
    state.testMotorVal[6] = table->GetNumber("Test Motor ID 11", 0.0);
    state.testMotorVal[7] = table->GetNumber("Test Motor ID 12", 0.0);
    state.testMotorVal[8] = table->GetNumber("Test Motor ID 13", 0.0);
    state.testMotorVal[9] = table->GetNumber("Test Motor ID 14", 0.0);

    table->PutBoolean("Test Photoelec Reading", state.photoelecReading);

}

void TestMotors::assignOutputs()
{
    for (int i = 0; i < 10; i++) {
        state.motors[i]->Set(state.testMotorVal[i]);
    }
       
}

void TestMotors::resetState()
{

}