/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.chaos131.auto.conditions;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Stops auto command if based on time. Requires int "timeMs" in parameter (in
 * milliseconds)
 */
public class TimeAutoCondition implements IAutoCondition {

    public static final String CONDITION_NAME = "timeMs";

    long m_startTimeMs; // stores current time at init
    int m_timeoutMs; // stores target time

    public TimeAutoCondition(int timeoutMs) {
        this.m_timeoutMs = timeoutMs;
    }

    @Override
    public void init() {
        m_startTimeMs = RobotController.getFPGATime() / 1000;
    }

    // when the total time is greater than 5, stop time auto command
    @Override
    public boolean isDone() {
        boolean result = ((RobotController.getFPGATime() / 1000) - m_startTimeMs) > m_timeoutMs;
        return result;
    }
}
