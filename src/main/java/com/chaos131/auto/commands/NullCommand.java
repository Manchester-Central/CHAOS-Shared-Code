/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.chaos131.auto.commands;

import com.chaos131.auto.ParsedCommand;

/**
 * Tells robot to do absolutely nothing. Immediately calls isFinished() Takes
 * zero parameters.
 */
public class NullCommand extends BaseAutoCommand {

    public NullCommand(ParsedCommand parsedCommand) {
        super(parsedCommand);
    }

    @Override
    protected boolean isWorkDone() {
        return true;
    }

    @Override
    protected void beforeWorking() {}

    @Override
    protected void work() {}

    @Override
    protected void afterWorking() {}
}
