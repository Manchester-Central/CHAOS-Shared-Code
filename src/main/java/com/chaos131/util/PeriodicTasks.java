// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

/**
 * This is a class designed to add automatic periodic processing of tasks, like checking for
 * dashboard number updates.
 */
public final class PeriodicTasks extends SubsystemBase {

  /** The singleton instance of the subsystem. */
  private static PeriodicTasks m_instance = new PeriodicTasks();

  /** Gets the singleton instance of the subsystem */
  public static PeriodicTasks getInstance() {
    return m_instance;
  }

  /** The list of the tasks to run. */
  private List<Runnable> m_tasks = new ArrayList<>();

  /** Creates a new PeriodicTasks. Private to force user of getInstance */
  private PeriodicTasks() {}

  /** Adds a new task to be processed periodically. */
  public void addTask(Runnable task) {
    m_tasks.add(task);
  }

  @Override
  public void periodic() {
    m_tasks.forEach(r -> r.run());
  }
}
