// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.gamepads;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.math.MathUtil;


/** Add your docs here. */
public class Gamepad {
    private Joystick m_joystick;
    private String m_controllerTitle;

    private Button m_buttonA;
    private Button m_buttonB;
    private Button m_buttonX;
    private Button m_buttonY;
    private Button m_buttonLB;
    private Button m_buttonLT;
    private Button m_buttonRB;
    private Button m_buttonRT;
    private Button m_buttonSelect;
    private Button m_buttonStart;

    private final XboxControllerMapping m_xboxControllerMapping = new XboxControllerMapping();
    private final LogitechControllerMapping m_logitechControllerMapping = new LogitechControllerMapping();
    private final WirelessControllerMapping m_wirelessControllerMapping = new WirelessControllerMapping();

    public Gamepad(int port, String controllerTitle) {
        m_joystick = new Joystick(port);
        m_controllerTitle = controllerTitle;

        m_buttonA = createButton(ButtonType.A);
        m_buttonB = createButton(ButtonType.B);
        m_buttonX = createButton(ButtonType.X);
        m_buttonY = createButton(ButtonType.Y);
        m_buttonLB = createButton(ButtonType.LeftBumper);
        m_buttonLT = createButton(ButtonType.LeftTrigger);
        m_buttonRB = createButton(ButtonType.RightBumper);
        m_buttonRT = createButton(ButtonType.RightTrigger);
        m_buttonSelect = createButton(ButtonType.Select);
        m_buttonStart = createButton(ButtonType.Start);

        addShuffleboeardTab();
    }

    private IControllerMapping getControllerMapping() {
        String joystickName = getControllerName().toLowerCase();
        switch (joystickName) {
            case "xbox controller":
                return m_xboxControllerMapping;
            case "wireless controller":
                return m_wirelessControllerMapping;
            default:
                return m_logitechControllerMapping;
        }
    }

    private String getControllerName() {
        return m_joystick.getName();
    }

    private double applyDeadband(double value) {
        return MathUtil.applyDeadband(value, 0.1);
    }

    public double getLeftX() {
        return applyDeadband(getControllerMapping().getLeftX(m_joystick));
    }

    public double getLeftY() {
        return applyDeadband(getControllerMapping().getLeftY(m_joystick));
    }

    public double getRightX() {
        return applyDeadband(getControllerMapping().getRightX(m_joystick));
    }

    public double getRightY() {
        return applyDeadband(getControllerMapping().getRightY(m_joystick));
    }

    public Button getButtonA() {
        return m_buttonA;
    }

    public Button getButtonB() {
        return m_buttonB;
    }

    public Button getButtonX() {
        return m_buttonX;
    }

    public Button getButtonY() {
        return m_buttonY;
    }

    public Button getButtonLB() {
        return m_buttonLB;
    }

    public Button getButtonLT() {
        return m_buttonLT;
    }

    public Button getButtonRB() {
        return m_buttonRB;
    }

    public Button getButtonRT() {
        return m_buttonRT;
    }

    public Button getButtonSelect() {
        return m_buttonSelect;
    }

    public Button getButtonStart() {
        return m_buttonStart;
    }

    private Button createButton(ButtonType buttonType) {
        return new Button() {
            public boolean get() {
                return getControllerMapping().getButtonPressed(m_joystick, buttonType);
            }
        };
    }

    private void addShuffleboeardTab() {
        ShuffleboardTab tab = Shuffleboard.getTab("Gamepad - " + m_controllerTitle);
        addButtonToDashboard(tab, "A", m_buttonA);
        addButtonToDashboard(tab, "B", m_buttonB);
        addButtonToDashboard(tab, "X", m_buttonX);
        addButtonToDashboard(tab, "Y", m_buttonY);
        addButtonToDashboard(tab, "LB", m_buttonLB);
        addButtonToDashboard(tab, "LT", m_buttonLT);
        addButtonToDashboard(tab, "RB", m_buttonRB);
        addButtonToDashboard(tab, "RT", m_buttonRT);
        addButtonToDashboard(tab, "Select", m_buttonSelect);
        addButtonToDashboard(tab, "Start", m_buttonStart);

        tab.addNumber("Left X", new DoubleSupplier(){
            public double getAsDouble() { return getLeftX(); };
        });

        tab.addNumber("Left Y", new DoubleSupplier(){
            public double getAsDouble() { return getLeftY(); };
        });

        tab.addNumber("Right X", new DoubleSupplier(){
            public double getAsDouble() { return getRightX(); };
        });

        tab.addNumber("Right Y", new DoubleSupplier(){
            public double getAsDouble() { return getRightY(); };
        });

        tab.addString("Controller Type --> Mapping", new Supplier<String>() {
            public String get() { return getControllerName() +  " --> " + getControllerMapping().getClass().getSimpleName(); }
        });
    }

    private void addButtonToDashboard(ShuffleboardTab tab, String buttonName, Button button) {
        tab.addBoolean(buttonName, new BooleanSupplier(){
            public boolean getAsBoolean() { return button.get(); };
        });
    }
}
