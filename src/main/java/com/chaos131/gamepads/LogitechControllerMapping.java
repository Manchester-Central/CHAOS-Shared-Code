// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.gamepads;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class maps the inputs for an Logitech style controller.
 * Tested with:
 *  - Logitech F310 Controller
 */
public class LogitechControllerMapping implements IControllerMapping {

    public double getLeftX(Joystick joystick) {
        return joystick.getRawAxis(0);
    }

    public double getLeftY(Joystick joystick) {
        return -joystick.getRawAxis(1);
    }

    public double getRightX(Joystick joystick) {
        return joystick.getRawAxis(2);
    }

    public double getRightY(Joystick joystick) {
        return -joystick.getRawAxis(3);
    }

    public boolean getButtonPressed(Joystick joystick, ButtonType buttonType) {
        switch (buttonType) {
            case A:
                return joystick.getRawButton(2);
            case B:
                return joystick.getRawButton(3);
            case X:
                return joystick.getRawButton(1);
            case Y:
                return joystick.getRawButton(4);
            case LeftBumper:
                return joystick.getRawButton(5);
            case LeftTrigger:
                return joystick.getRawButton(7);
            case RightBumper:
                return joystick.getRawButton(6);
            case RightTrigger:
                return joystick.getRawButton(8);
            case Select:
                return joystick.getRawButton(9);
            case Start:
                return joystick.getRawButton(10);
            case LeftStick:
                return joystick.getRawButton(11);
            case RightStick:
                return joystick.getRawButton(12);
            default:
                System.err.println("Button type not supported: " + buttonType.toString());
                return false;
        }
    }

}
