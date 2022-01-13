// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.gamepads;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class maps the inputs for an Xbox style controller.
 * Tested with:
 *  - Xbox One Controller (do NOT enable 'Map Gamepad' in simulator)
 */
public class XboxControllerMapping implements IControllerMapping {

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
                return joystick.getRawButton(1);
            case B:
                return joystick.getRawButton(2);
            case X:
                return joystick.getRawButton(3);
            case Y:
                return joystick.getRawButton(4);
            case LeftBumper:
                return joystick.getRawButton(5);
            case LeftTrigger:
                return joystick.getRawAxis(4) > 0.5;
            case RightBumper:
                return joystick.getRawButton(6);
            case RightTrigger:
                return joystick.getRawAxis(5) > 0.5;
            case Select:
                return joystick.getRawButton(7);
            case Start:
                return joystick.getRawButton(8);
            default:
                System.err.println("Button type not supported: " + buttonType.toString());
                return false;
        }
    }

}
