package com.chaos131.util;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Simple syntactic sugar to automatically trigger runnables when a chooser changes state. Ex.
 *
 * <pre>{@code var chooser = new DashboardActions("TopicGroup/Topic",
 *  "Enable", () -> robotFlag = true);
 * chooser.addOption("Disable", () -> robotFlag = false);
 * return chooser;}</pre>
 */
public class DashboardActions extends LoggedDashboardChooser<Runnable> {
  public DashboardActions(String name, String default_action_name, Runnable default_task) {
    super(name);
    addOption(default_action_name, default_task);
    onChange(this::doAction);
  }

  private void doAction(Runnable task) {
    task.run();
  }
}
