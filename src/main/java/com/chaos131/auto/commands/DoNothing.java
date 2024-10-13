/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.chaos131.auto.commands;

import com.chaos131.auto.ParsedCommand;

/**
 * Does nothing - designed for waiting a certain amount of time. Won't ever end unless an IAutoCondition is added
 */
public class DoNothing extends BaseAutoCommand {

    /** Empty command name */
    public static final String COMMAND_NAME = "wait";

    /**
     * Does nothing, really.
     * @param parsedCommand doesn't even matter
     */
    public DoNothing(ParsedCommand parsedCommand) {
        super(parsedCommand);
    }

    @Override
    protected boolean isWorkDone() {
        return false;
    }

    @Override
    protected void beforeWorking() {}

    @Override
    protected void work() {}

    @Override
    protected void afterWorking() {}

}
