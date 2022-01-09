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

class ValorSubsystem : public frc2::Subsystem {
    public:
        ValorSubsystem();
        
        void Periodic();
        
        static ValorSubsystem& GetInstance();
        
        // should initialize subsystem's devices
        virtual void init();

        // Rules:
        //   * Only intended to sync state to the dasboard
        virtual void analyzeDashboard();
        
        // Rules:
        //   * Never read 'state', can only write 'state'
        virtual void assessInputs();
        
        // Rules:
        //   * Never write another subsystem's 'state', only can read
        //   * Can read or write 'state'
        virtual void assignOutputs();
        
        // should reset the subsystem state to robot setup position
        virtual void resetState();

        enum RobotMode {
            DISABLED,
            AUTO,
            TELEOP
        } robotMode;
    
    protected:

        void initTable(const char* name);

        std::shared_ptr<nt::NetworkTable> table;
};