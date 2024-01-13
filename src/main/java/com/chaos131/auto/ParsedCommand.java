// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.auto;

import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

/**
 * Takes Value format `commandName?arg1{@literal &}arg2...` and separates it. :)
 * 
 * Copied from 2020 code: 
 *   - https://github.com/Manchester-Central/2020-Infinite-Recharge/blob/master/src/main/java/frc/robot/auto/ParseCommand.java
 */
public class ParsedCommand {

    String commandName;
    String originalString;
    // List that contains key and value as 2 strings (like an array)
    // think of it as an array, but indexes are names, not numbers
    Map<String, String> arguments = new TreeMap<>(String.CASE_INSENSITIVE_ORDER);

    public ParsedCommand(String commandString) {
        originalString = commandString;

        // split separates commandName from arguments
        String[] splitStrings = commandString.split("\\?");

        // assigns commandName to left of split (array 0)
        commandName = splitStrings[0];
        System.out.println("CommandName: " + commandName);

        // if no arguments, stop
        if (splitStrings.length < 2) {
            return;
        }

        // stores right side of split for further use
        String rightSide = splitStrings[1];

        // for each loop: for every & symbol in string, split into new chunk and assign
        // to array
        for (String argString : rightSide.split("&")) {
            // for every "=" symbol in the & chunk, split into new chunk and assign to array
            // splitArgs
            String[] splitArgs = argString.split("=");
            // adds the arguments to map list arguments above
            arguments.put(splitArgs[0], splitArgs[1]);

            // Checking to see if code works
            System.out.println("Arguments: " + splitArgs[0]);
            System.out.println(arguments.get(splitArgs[0]));
        }

    }

    /***
     * returns arguments assigned from for each loop, found in map arguments
     *
     * @param key
     * @return arguments in specific key
     */
    public String getArgument(String key) {

        return arguments.get(key);

    }

    public Set<String> getArgumentKeys() {
        return arguments.keySet();
    }

    @Override
    public String toString() {
        return originalString;
    }
}
