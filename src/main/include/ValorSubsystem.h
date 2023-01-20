/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>

#include <frc/TimedRobot.h>

#include "ValorGamepad.h"

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableHelper.h>

/**
 * @brief Abstract class that all Valor subsystem's should implement
 * 
 * To make developer's lives easier and to prevent any mistakes in a quick build season,
 * ValorSubsystem is used to organize code and abstract a lot of the base code that is often
 * repetitive in all subsystems.
 * 
 * Valor Subsystem implements the FRC WPILib Subsystem class and therefore has all baked in
 * features from WPILib.
 * 
 * The idea is that subsystems on the robot implement ValorSubsystem and logic for that
 * subsystem is broken out into 3 different functions:
 * * assessInputs
 * * analyzeDashboard
 * * assignOutputs
 * 
 * Every 20 milliseconds when the robot is on, the functions will run in order if the robot
 * state matches what is in the table below:
 * Robot State | assessInputs | analyzeDashboards | assignOutputs
 * ------------|--------------|-------------------|--------------
 * Disabled    | <pre>✖</pre> | <pre>✔</pre>     | <pre>✖</pre>        
 * Auto        | <pre>✖</pre> | <pre>✔</pre>     | <pre>✔</pre>         
 * Teleop      | <pre>✔</pre> | <pre>✔</pre>     | <pre>✔</pre>         
 * 
 * Descriptions for each function and their intent is listed in the function description
 * 
 */
class ValorSubsystem : public frc2::Subsystem, public wpi::Sendable, public wpi::SendableHelper<ValorSubsystem> {
    public:

        /**
         * @brief Construct a new Valor Subsystem object
         * 
         * Sets up the infrastructure so that the various robot modes will automatically
         * call the proper function. This NEEDS to be called from the constructor of every
         * subsystem. This will force the constructor of every subsystem to also contain
         * a pointer to the TimedRobot instance (aka. Robot since Robot implements TimedRobot).
         * 
         * @param _robot Pass in the Robot reference so robot state can be auto-determined
         * @param name A human readable name of the subsystem 
         */
        ValorSubsystem(frc::TimedRobot *_robot, const char* name) :
            robot(_robot), subsystemName(name),
            operatorGamepad(NULL),
            driverGamepad(NULL)
        {
            table = nt::NetworkTableInstance::GetDefault().GetTable(name);
            wpi::SendableRegistry::AddLW(this, "ValorSubsystem", subsystemName);
            init();
        }
        
        /**
         * @brief Initialize the subsystem
         * 
         * Used to setup motors, sensors, and state. All setup code should go in this function.
         * The default init function contains no code. 
         * 
         * This function is a virtual function and should be implemented by the subsystem.
         */
        virtual void init() {}

        /**
         * @brief Synchronize dashboard data (both read and write)
         * 
         * The analyzeDashboard function runs at all times (disabled, auto, and teleop).
         * The intent of the function is to send sensor and state information from the robot
         * to the Driver Station for debugging. Additionally, driver commands and data can be
         * sent from the Driver Station to the robot and collected in this function.
         * 
         * ALL network table and sensor logic should be in this function as both network table
         * and sensor information should be read at all times, not just during teleop since auto
         * will never get that information.
         * 
         * This function is a virtual function and should be implemented by the subsystem.
         */
        virtual void analyzeDashboard() {}
        
        /**
         * @brief Read controller logic and set subsystem state
         * 
         * assessInputs only runs during teleop and should be used to convert driver/operator
         * inputs into subsystem state. Since this function only runs during teleop, driver/operator
         * inputs in auto are ignored which is completely okay. Instead, our auto commands will
         * mimic driver input and set the states.
         * 
         * Note that state should not be READ in this function, and instead should only be WRITEs.
         * This is because using previous state will occur in assignOutputs when determining what
         * values to send to motors.
         * 
         * Suggested flow:
         * * Read inputs from driver/operator
         * * Set state based on those inputs in if statements
         *   * The higher up in the if statement, the higher the priority of the state
         * 
         * This function is a virtual function and should be implemented by the subsystem.
         */
        virtual void assessInputs() {}

        /**
         * @brief Read subsystem state and send motor commands
         * 
         * This function runs during teleop and auto, but not disabled. This is because
         * the function is intended to control motors and should not send motor outputs when
         * the robot is disabled. assignOutputs should read the subsystem state set in
         * assessInputs and analyzeDashboard and create motor commands.
         * 
         * Note that state should be READ and not be written to (the opposite of assessInputs).
         * The big rule to obey with this function is that other subsystem's state should never
         * be written to. For example, if sensors in subsystem A determine outputs for subsystem
         * A AND subsystem B, then the sensor should live in both subsystems instead of subsystem
         * A trying to write results in subsystem B. This will make logic easier to understand for
         * all developers.
         * 
         * Suggested flow:
         * * Read subsystem state
         * * Set motor output based on those states in if statements
         *   * The higher up in the if statement, the higher the priority of the motor output
         * 
         * This function is a virtual function and should be implemented by the subsystem.
         */
        virtual void assignOutputs() {}
        
        /**
         * @brief Reset all subsystem state
         * 
         * Use this function to set the subsystem state to default values,
         * as well as set the motor outputs to 0 so the robot doesn't "lurch" when booting up
         * due to phantom state (aka. state that was previously set but hasn't been reset yet).
         * 
         * This function is a virtual function and should be implemented by the subsystem.
         */
        virtual void resetState() {}

        void setGamepads(ValorGamepad *_operatorGamepad, ValorGamepad *_driverGamepad)
        {
            operatorGamepad = _operatorGamepad;
            driverGamepad = _driverGamepad;
        }

        virtual void InitSendable(wpi::SendableBuilder& builder) = 0;
    
    protected:

        /**
         * @brief SmartDashboard table associated with the subsystem
         * 
         * Unique table for the subsystem so that the subsystem can read and write data to
         * the smart dashboard.
         */
        std::shared_ptr<nt::NetworkTable> table;

        frc::TimedRobot *robot;
        const char* subsystemName;

        ValorGamepad *operatorGamepad;
        ValorGamepad *driverGamepad;

    private:
        
        void Periodic()
        {       
            if (robot->IsTeleop())
                assessInputs();

            analyzeDashboard();

            if (!robot->IsDisabled())
                assignOutputs();
        }
};