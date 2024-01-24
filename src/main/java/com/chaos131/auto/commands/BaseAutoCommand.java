/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.chaos131.auto.commands;

import com.chaos131.auto.ParsedCommand;
import com.chaos131.auto.conditions.IAutoCondition;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.ArrayList;
import java.util.List;

/**
 * A class for handling the logic needed for our Auto commands
 */
public abstract class BaseAutoCommand extends CommandBase {

    // a list (advanced array) of called conditions
    // the conditions in this list will depend on the parameters set in "value" of SmartDashboard
    private List<IAutoCondition> m_conditions = new ArrayList<IAutoCondition>();
    private long m_startTimeMs;

    protected ParsedCommand m_parsedCommand;

    public BaseAutoCommand(ParsedCommand parsedCommand) {
        m_parsedCommand = parsedCommand;
    }

    public void addCondition(IAutoCondition condition) {
        m_conditions.add(condition);
    }

    @Override
    public final void initialize() {
        m_startTimeMs = RobotController.getFPGATime() / 1000;
        System.out.println("Starting auto command: " + m_parsedCommand);
        super.initialize();
        for (IAutoCondition condition : m_conditions) {
            condition.init();
        }
        beforeWorking();
    }

    @Override
    public final boolean isFinished() {
        for (IAutoCondition condition : m_conditions) {
            if (condition.isDone()) {
                return true;
            }
        }
        return isWorkDone();
    }

    @Override
    public final void end(boolean interrupted) {
        super.end(interrupted);
        afterWorking();
        var elapsedTimeMs = (RobotController.getFPGATime() / 1000) - m_startTimeMs;
        System.out.println("Auto command finished after " + elapsedTimeMs + "ms");
    }

    @Override
    public final void execute() {
        super.execute();
        work();
    }

    /**
     * Should be implemented by each command.
     * Will record if the work for the command is done, but can be overridden by configured IAutoConditions
     * @return true if the command is finished
     */
    protected abstract boolean isWorkDone();

    /**
     * Implement code here that should happen before the command starts
     */
    protected abstract void beforeWorking();

    /**
     * Implement code here for what should happen during the command
     */
    protected abstract void work();

    /**
     * Implement code here for what should happen after a command
     */
    protected abstract void afterWorking();
}
