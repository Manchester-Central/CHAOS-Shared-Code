package com.chaos131.robot;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.swerve.BaseSwerveDrive;
import com.chaos131.vision.VisionData;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Base robot container that accompanies ChaosRobot, and contains many data structures and methods
 * that are common across seasons and projects. Must pass in the type of DriveSystem as a generic
 * since every year we change it slightly (new dependency feature, )
 */
public abstract class ChaosRobotContainer<T extends BaseSwerveDrive> {
  /** Default Constructor, initializes AutoChoices, Controller Setups */
  public ChaosRobotContainer() {
    // PathPlanner Setup
    configureDriveSystem();
    configureDriverController();
    configureOperatorController();
    configureSimKeyboard();
    configureTesterController();
    addAutoChoices();
    buildPathplannerAutoChooser();
  }

  /**
   * PathPlanner must be setup (presumably in configureDriveSystem) before we can build the auto
   * chooser that interacts with smart dashboard. Which is weird but it is what it is I suppose.
   */
  protected void buildPathplannerAutoChooser() {
    if (AutoBuilder.isConfigured()) {
      m_pathPlannerChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", m_pathPlannerChooser);
    }
  }

  /***************************
   *    System Structures    *
   ***************************/

  /** PathPlanner data structure */
  private SendableChooser<Command> m_pathPlannerChooser;

  /** Swerve Drive System */
  protected T m_swerveDrive;

  /************************
   *    Control Inputs    *
   ************************/

  /** Controller for stearing */
  protected Gamepad m_driver;

  /** Controller for additional features */
  protected Gamepad m_operator;

  /** Keyboard controller setup for testing without a controller */
  protected Gamepad m_simKeyboard;

  /** Controller for testing experimental features */
  protected Gamepad m_tester;

  /** Presumably this is a Swerve Drive system */
  protected abstract void configureDriveSystem();

  /** Required to be implemented by child class to setup the Driver's controller interface. */
  protected abstract void configureDriverController();

  /** Required to be implemented by child class to setup the Operators's controller interface. */
  protected abstract void configureOperatorController();

  /** Required to be implemented by child class to setup a Tester controller interface. */
  protected abstract void configureTesterController();

  /** Required to be implemented by child class to setup a keyboard simulation interface. */
  protected abstract void configureSimKeyboard();

  /**************************
   *    Autonomous Setup    *
   **************************/

  /** Implemented by child classes to list all the commands we implement. */
  protected abstract void addAutoChoices();

  /**
   * Adds a choice to the dropdown for PathPlanner
   *
   * @param name of the cmd in the list
   * @param cmd actual command object to be used
   */
  protected void addAutoChoice(String name, Command cmd) {
    NamedCommands.registerCommand(name, cmd);
  }

  /**
   * @return the command choosen according to network tables
   */
  public Command getAutonomousCommand() {
    if (m_pathPlannerChooser == null) return null;
    return m_pathPlannerChooser.getSelected();
  }

  /** Run periodically by the ChaosRobot class */
  public abstract void periodic();

  /**
   * Attempts to update the pose estimator within the swerve drive object. Note that the SwerveDrive
   * may disregard pose updates as well.
   *
   * @param data VisionData structure containing the required parts
   */
  public synchronized void updatePoseEstimator(VisionData data) {
    var pose = data.getPose2d();
    if (pose == null
        || !Double.isFinite(pose.getX())
        || !Double.isFinite(pose.getY())
        || !Double.isFinite(pose.getRotation().getDegrees())) {
      return;
    }
    m_swerveDrive.addVisionMeasurement(data);
  }
}
