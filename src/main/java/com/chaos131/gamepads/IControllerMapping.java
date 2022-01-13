// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.gamepads;

import edu.wpi.first.wpilibj.Joystick;

public interface IControllerMapping {
    double getLeftX(Joystick joystick);

    double getLeftY(Joystick joystick);

    double getRightX(Joystick joystick);

    double getRightY(Joystick joystick);

    boolean getButtonPressed(Joystick joystick, ButtonType buttonType);
}
