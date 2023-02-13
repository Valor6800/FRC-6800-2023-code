/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "ValorPIDF.h"
#include "ValorNeutralMode.h"

#include <string>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableHelper.h>

/**
 * @brief Abstract class that all Valor controllers's should implement
 * @tparam T Motor data type
 * 
 * To make developer's lives easier and to prevent any mistakes in a quick build season,
 * ValorController is used to organize code and abstract a lot of the base code that is often
 * repetitive in all motor controllers.
 * 
 * The idea is that motor controllers on the robot implement ValorController and logic for that
 * motor controller is run by the implemented class. Setup for the motors should also occur
 * in the implemented ValorController, and the motor pointer itself also lives in this class.
 * 
 * Helper methods exist to make it easier for subsystems to control motors in a variety of ways.
 * 
 * Usage:
 * \code {.cpp}
 * public class ValorFalconController : public ValorController<WPI_TalonFX> { };
 * \endcode
 */
template <class T>
class ValorController : public wpi::Sendable, public wpi::SendableHelper<ValorController<T>>
{
public:

    /**
     * @brief Construct a new Valor Controller object
     * 
     * @param _motor The motor that will be controlled. Setup by the implemented class
     */
    ValorController(T* _motor, bool _inverted, ValorNeutralMode _neutralMode) :
        motor(_motor),
        inverted(_inverted),
        neutralMode(_neutralMode),
        conversion(1) {}

    /**
     * @brief Destroy the Valor Controller object
     * 
     * Due to the ValorController implementation owning the motor, a destructor
     * is needed to clean up the motor and any followers. The motor and follower
     * are stored on the heap so use the delete keyword to remove them from the heap.
     */
    ~ValorController()
    {
        if (motor) {
            delete motor;
            motor = nullptr;
        };
        if (followerMotor) {
            delete followerMotor;
            followerMotor = nullptr;
        };
    }

    /**
     * @brief Get a pointer to the Motor object that the ValorController implementation owns
     * 
     * @return T* Pointer to the motor object. Allows developers to call functions attached to the motor
     */
    T* getMotor() { return motor; }

    /**
     * @brief Initialize the motor. Setup any parameters that get burned to the motor's flash memory
     * 
     * Usually used to setup motor parameters, and called via the constructor.
     * Needs to be defined in the implemented class
     * 
     * To be defined by the implemented ValorController class
     */
    virtual void init() = 0;

    /**
     * @brief Resets the motor and any state
     * 
     * Clear the encoders for the motor and set to 0.
     * 
     * Additionally, should be called by the constructor to set default values
     * before any logic is run.
     * 
     * To be defined by the implemented ValorController class
     */
    virtual void reset() = 0;

    /**
     * @brief Get the motors position
     * 
     * Units by default are in rotations of the motor shaft.
     * To change the units that this function returns, use @link setConversion @endlink.
     * 
     * To be defined by the implemented ValorController class
     * 
     * @return double Position of the motor
     */
    virtual double getPosition() = 0;

    /**
     * @brief Get the motors output current
     * 
     * Get the instantaneous amperage of the motor that the controller owns
     * 
     * @return double Instantaneous amperage of the motor
     */
    virtual double getCurrent() = 0;

    /**
     * @brief Get the motors speed
     * 
     * Units by default are in rotations per second of the motor shaft.
     * To change the units that this function returns, use @link setConversion @endlink.
     * 
     * To be defined by the implemented ValorController class
     * 
     * @return double Speed of the motor
     */
    virtual double getSpeed() = 0;
    
    virtual void setEncoderPosition(double position) = 0;

    /**
     * @brief Send the motor to a specific position
     * 
     * Units by default are in rotations of the motor shaft.
     * Will use the motor's native trapezoidal motion profile to get the motor to that position.
     * Can be tuned using the velocity and acceleration components of ValorPIDF via @link setPIDF @endlink
     * 
     * To be defined by the implemented ValorController class
     * 
     * @param position The position to send the motor to
     */
    virtual void setPosition(double position) = 0;

    /**
     * @brief Send the motor to a specific speed
     * 
     * Units by default are in rotations per second of the motor shaft.
     * Will use the motor's native trapezoidal motion profile to get the motor to that position.
     * Can be tuned using the PIDF components of ValorPIDF via @link setPIDF @endlink
     * 
     * To be defined by the implemented ValorController class
     * 
     * @param speed The speed to set the motor to
     */
    virtual void setSpeed(double speed) = 0;

    /**
     * @brief Set the motor power
     * 
     * Use the traditional -1 to 1 values to set the motor power.
     * Note that if voltage compensation is on, 1 will not equal 12V
     * 
     * To be defined by the implemented ValorController class
     * 
     * @param power The power to set the motor to
     * 
     */
    virtual void setPower(double power) = 0;

    /**
     * @brief If a motor is paired with another motor, setup that other motor as a follower
     * 
     * The follower motor will need a CAN ID, and then it will mimic and assume
     * all other parameters of the lead motor
     * 
     * To be defined by the implemented ValorController class
     * 
     * @param canID The CAN ID of the follower motor
     */
    virtual void setupFollower(int canID, bool followerInverted = false) = 0;
    
    /**
     * @brief Change the PIDF values for the motor
     * 
     * ValorPIDF has some default values already set and tested, but if the system
     * requires some changes, use this method to change those defaults.
     * 
     * @param pidf The new PIDF values to use for the system
     * @param slot Set which slot of the motor to apply the PIDF. 0 if slots aren't compatible
     */
    virtual void setPIDF(ValorPIDF pidf, int slot = 0) = 0;

    /**
     * @brief Set both soft limits for the motor
     * 
     * Soft limits restrict the reverse and forward direction to a certain range.
     * Units by default are in rotations of the motor shaft.
     * 
     * Calls out to the pure virtual functions @link setForwardLimit @endlink
     * and @link setReverseLimit @endlink
     * 
     * @param reverse The reverse soft limit
     * @param forward The forward soft limit
     */
    void setLimits(int reverse, int forward)
    {
        setForwardLimit(forward);
        setReverseLimit(reverse);
    }

    /**
     * @brief Set the forward soft limit for the motor
     * 
     * Soft limits restrict the reverse and forward direction to a certain range.
     * Units by default are in rotations of the motor shaft.
     * 
     * @param forward The forward soft limit
     */
    virtual void setForwardLimit(double forward) = 0;

    /**
     * @brief Set the reverse soft limit for the motor
     * 
     * Soft limits restrict the reverse and reverse direction to a certain range.
     * Units by default are in rotations of the motor shaft.
     * 
     * @param reverse The reverse soft limit
     */
    virtual void setReverseLimit(double reverse) = 0;

    /**
     * @brief Set the power range of the motor
     * 
     * Only used when @link setPower @endlink has been used.
     * This will restrict the minimum and maximum power that can be used.
     * 
     * To be defined by the implemented ValorController class
     * 
     * @param slot The slot to configure the min/max for
     * @param min Minimum power to use (between 0 and 1)
     * @param max Maximum power to use (between 0 and 1)
     */
    virtual void setRange(int slot, double min, double max) = 0;

    /**
     * @brief Set the conversion factor for all methods
     * 
     * Used to convert between rotations and whatever units the developer requires.
     * Say the developer wants to control the motor in degrees, and there is a 5:1 gearbox on the motor.
     * Then the conversion factor would be (360.0 / 5 ), which says for every 5 rotations of the motor,
     * that yields 360 degrees.
     * 
     * Any easy way to find this conversion factor is through dimensional analysis:
     * X motor rotations | 1 output rotation | 360 degrees 
     * ------------------|-------------------|-------------
     *          -        | 5 motor rotations | 1 output rotation
     * 
     * To be defined by the implemented ValorController class
     * 
     * @param conversion The conversion factor to apply to all calculations
     */
    virtual void setConversion(double conversion) = 0;

    /**
     * @brief Set which profile to use
     * 
     * Multiple motor profiles can be setup. This method chooses which motor profile is active
     * 
     * To be defined by the implemented ValorController class
     * 
     * @param slot Which profile to turn active
     */
    virtual void setProfile(int slot) = 0;

    virtual void setNeutralMode(ValorNeutralMode mode) = 0;

    virtual void InitSendable(wpi::SendableBuilder& builder) = 0;

    virtual double getAbsEncoderPosition() = 0;

protected:

    T* motor;
    T* followerMotor;

    double conversion;

    bool inverted;

    ValorNeutralMode neutralMode;
};