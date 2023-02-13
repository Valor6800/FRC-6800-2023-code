/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/XboxController.h>

/**
 * @brief Wrapper class for XBox gamepads
 * 
 * Used to enhance the default methods in the FRC XboxController class.
 * Additional methods are needed to make gamepad utilization easier and
 * cleaner in subsystem classes.
 * 
 * Focus on providing more resolution and functions for X/Y thumb-pads.
 * 
 */
class ValorGamepad : public frc::XboxController
{
public:

    /**
     * @brief Construct a new Valor Gamepad object
     * 
     * @param id USB ID that shows up on the driver station. Between 0 and 3
     */
    ValorGamepad(int id);

    /**
     * @brief Set the X deadband value
     * 
     * Deadband is the range of values where where the values can be varied without 
     * changing the output response.
     * 
     * For example, if the deadband is 0.1, then any values between -0.1 and 0.1 will
     * all yield an output response of 0.
     * 
     * @param deadband Deadband value for the X direction of the gamepad. Between 0 and 1
     */
    void setDeadbandX(double deadband);

    /**
     * @brief Set the Y deadband value
     * 
     * Deadband is the range of values where where the values can be varied without 
     * changing the output response.
     * 
     * For example, if the deadband is 0.1, then any values between -0.1 and 0.1 will
     * all yield an output response of 0.
     * 
     * @param deadband Deadband value for the Y direction of the gamepad. Between 0 and 1
     */
    void setDeadbandY(double deadband);

    /**
     * @brief Obtain the X value from the gamepad's left thumb stick
     * 
     * First, the gamepad's raw X value is compared against the set deadband. See the
     * @link setDeadbandX @endlink function. If the input is inside the deadband, the output
     * is set to 0. Otherwise, the gamepad's raw X value is raised to the power of the input polynomial.
     * For example, if polynomial parameter is 2, then the input is squared (x^2).
     * 
     * @param polynomial What to raise the input to the power of
     * @return double X value of the gamepad's left thumb stick. Between -1 and 1
     */
    double leftStickX(int polynomial=1);

    /**
     * @brief Is the gamepad's left X thumb stick active?
     * 
     * @param polynomial The exponent to apply to the gamepad's stick value
     * @return true Gamepad's left X thumb stick is outside of deadband
     * @return false Gamepad's left X thumb stick is at rest and within deadband
     */
    bool leftStickXActive(int polynomial=1);
    
    /**
     * @brief Obtain the Y value from the gamepad's left thumb stick
     * 
     * First, the gamepad's raw Y value is compared against the set deadband. See the
     * @link setDeadbandY @endlink function. If the input is inside the deadband, the output
     * is set to 0. Otherwise, the gamepad's raw Y value is raised to the power of the input polynomial.
     * For example, if polynomial parameter is 2, then the input is squared (x^2).
     * 
     * @param polynomial The exponent to apply to the gamepad's stick value
     * @return double Y value of the gamepad's left thumb stick. Between -1 and 1
     */
    double leftStickY(int polynomial=1);

    /**
     * @brief Is the gamepad's left Y thumb stick active?
     * 
     * @param polynomial The exponent to apply to the gamepad's stick value
     * @return true Gamepad's left Y thumb stick is outside of deadband
     * @return false Gamepad's left Y thumb stick is at rest and within deadband
     */
    bool leftStickYActive(int polynomial=1);

    /**
     * @brief Obtain the X value from the gamepad's right thumb stick
     * 
     * First, the gamepad's raw X value is compared against the set deadband. See the
     * @link setDeadbandX @endlink function. If the input is inside the deadband, the output
     * is set to 0. Otherwise, the gamepad's raw X value is raised to the power of the input polynomial.
     * For example, if polynomial parameter is 2, then the input is squared (x^2).
     * 
     * @param polynomial What to raise the input to the power of
     * @return double X value of the gamepad's right thumb stick. Between -1 and 1
     */
    double rightStickX(int polynomial=1);

    /**
     * @brief Is the gamepad's right X thumb stick active?
     * 
     * @param polynomial The exponent to apply to the gamepad's stick value
     * @return true Gamepad's right X thumb stick is outside of deadband
     * @return false Gamepad's right X thumb stick is at rest and within deadband
     */
    bool rightStickXActive(int polynomial=1);
    
    /**
     * @brief Obtain the Y value from the gamepad's right thumb stick
     * 
     * First, the gamepad's raw Y value is compared against the set deadband. See the
     * @link setDeadbandY @endlink function. If the input is inside the deadband, the output
     * is set to 0. Otherwise, the gamepad's raw Y value is raised to the power of the input polynomial.
     * For example, if polynomial parameter is 2, then the input is squared (x^2).
     * 
     * @param polynomial The exponent to apply to the gamepad's stick value
     * @return double Y value of the gamepad's right thumb stick. Between -1 and 1
     */
    double rightStickY(int polynomial=1);

    /**
     * @brief Is the gamepad's right Y thumb stick active?
     * 
     * @param polynomial The exponent to apply to the gamepad's stick value
     * @return true Gamepad's right Y thumb stick is outside of deadband
     * @return false Gamepad's right Y thumb stick is at rest and within deadband
     */
    bool rightStickYActive(int polynomial=1);

    /**
     * @brief Obtain the gamepad's right trigger value
     * 
     * @return double Gamepad's right trigger value. Between 0 and 1
     */
    double rightTrigger();

    /**
     * @brief Is the gamepad's right trigger active?
     * 
     * @return true Gamepad's right trigger is pressed to some degree
     * @return false Gamepad's right trigger is at rest
     */
    bool rightTriggerActive();

    /**
     * @brief Obtain the gamepad's left trigger value
     * 
     * @return double Gamepad's left trigger value. Between 0 and 1
     */
    double leftTrigger();

    /**
     * @brief Is the gamepad's left trigger active?
     * 
     * @return true Gamepad's left trigger is pressed to some degree
     * @return false Gamepad's left trigger is at rest
     */
    bool leftTriggerActive();

    /**
     * @brief Is the gamepad's D-pad up button pressed?
     * 
     * @return true Gamepad's up d-pad button is pressed
     * @return true Gamepad's up d-pad button is not pressed
     */
    bool DPadUp();

    /**
     * @brief Is the gamepad's D-pad down button pressed?
     * 
     * @return true Gamepad's down d-pad button is pressed
     * @return true Gamepad's down d-pad button is not pressed
     */
    bool DPadDown();

    /**
     * @brief Is the gamepad's D-pad left button pressed?
     * 
     * @return true Gamepad's left d-pad button is pressed
     * @return true Gamepad's left d-pad button is not pressed
     */
    bool DPadLeft();

    /**
     * @brief Is the gamepad's D-pad right button pressed?
     * 
     * @return true Gamepad's right d-pad button is pressed
     * @return true Gamepad's right d-pad button is not pressed
     */
    bool DPadRight();

private:
    double deadband(double, double, int);

    double deadbandX;
    double deadbandY;
};