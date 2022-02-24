// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.auto;

import java.util.Map;
import java.util.TreeMap;
import java.util.function.Function;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.chaos131.auto.commands.BaseAutoCommand;
import com.chaos131.auto.commands.DoNothing;
import com.chaos131.auto.commands.NullCommand;
import com.chaos131.auto.conditions.IAutoCondition;
import com.chaos131.auto.conditions.TimeAutoCondition;

/** This is the AutoBuilder that we use to run our simple Auto Scripts. 
 * 
 * This class was originally made by students for our 2020 robot: 
 *  - https://github.com/Manchester-Central/2020-Infinite-Recharge/blob/master/src/main/java/frc/robot/auto/AutoBuilder.java
*/
public class AutoBuilder {
    private final String k_KeyName = "Auto Steps";
    private SequentialCommandGroup m_commandList = new SequentialCommandGroup(); // sequence of commands to run
    private Map<String, Function<ParsedCommand, Command>> m_knownCommands = new TreeMap<>(String.CASE_INSENSITIVE_ORDER);
    private Map<String, Function<String, IAutoCondition>> m_knownConditions = new TreeMap<>(String.CASE_INSENSITIVE_ORDER);

    public AutoBuilder() {
        var defaultAuto = new String[] { "No Auto Selected" };
        SmartDashboard.setPersistent(k_KeyName);
        var lastAuto = SmartDashboard.getStringArray(k_KeyName, new String[]{});
        if (lastAuto.length == 0) {
            lastAuto = defaultAuto;
        }
        SmartDashboard.putStringArray(k_KeyName, lastAuto);
        registerCommand(DoNothing.COMMAND_NAME, (ParsedCommand parsedCommand) -> new DoNothing(parsedCommand));
        registerCondition(TimeAutoCondition.CONDITION_NAME, (String timeMs) -> new TimeAutoCondition(Integer.parseInt(timeMs)));
    }

    public AutoBuilder registerCommand(String commandName, Function<ParsedCommand, Command> commandGetter) {
        m_knownCommands.put(commandName, commandGetter);
        return this;
    }

    public AutoBuilder registerCondition(String conditionName, Function<String, IAutoCondition> conditionGetter) {
        m_knownConditions.put(conditionName, conditionGetter);
        return this;
    }

    public Command createAutoCommand() {
        m_commandList = new SequentialCommandGroup();
        var steps = SmartDashboard.getStringArray(k_KeyName, new String[]{});
        for (var step: steps) {
            System.out.println(step);
            var parsedCommand = new ParsedCommand(step);
            var command = this.getCommand(parsedCommand);
            m_commandList.addCommands(command);
            System.out.printf("Added command %s for %s\n", command.getClass(), step);
        }
        return m_commandList;
    }

    // depending on the arguments, creates new command
    private Command getCommand(ParsedCommand parsedCommand) {
        try {
            if (m_knownCommands.containsKey(parsedCommand.commandName)) {
                var command = m_knownCommands.get(parsedCommand.commandName).apply(parsedCommand);
                var autoBaseCommand = (BaseAutoCommand) null;
                if(command instanceof BaseAutoCommand) {
                    autoBaseCommand = (BaseAutoCommand) command;
                } else {
                    autoBaseCommand = new DoNothing(parsedCommand);
                    command = command.raceWith(autoBaseCommand);
                }
                for(var key: parsedCommand.getArgumentKeys()) {
                    if(m_knownConditions.containsKey(key)) {
                        autoBaseCommand.addCondition(m_knownConditions.get(key).apply(parsedCommand.getArgument(key)));
                    }
                }
                return command;
            }
            System.err.println("No matching command found for: " + parsedCommand);
        } catch(Exception ex) {
            System.err.println("Failed to create command for: " + parsedCommand);
            ex.printStackTrace();
        }
        return new NullCommand(parsedCommand);
    }
}
