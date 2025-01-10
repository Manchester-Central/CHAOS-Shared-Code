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
 * that are common across seasons and projects.
 */
public abstract class ChaosRobotContainer {
  /** Default Constructor, initializes AutoChoices, Controller Setups */
  public ChaosRobotContainer() {
    // PathPlanner Setup
    addAutoChoices();
    configureDriverController();
    configureOperatorController();
    configureSimKeyboard();
    configureTesterController();
  }

  protected void BuildAutoer() {
    if (AutoBuilder.isConfigured()){
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
  protected BaseSwerveDrive m_swerveDrive;

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

  /** Required to be implemented by child class to setup the Driver's controller interface. */
  public abstract void configureDriverController();

  /** Required to be implemented by child class to setup the Operators's controller interface. */
  public abstract void configureOperatorController();

  /** Required to be implemented by child class to setup a Tester controller interface. */
  public abstract void configureTesterController();

  /** Required to be implemented by child class to setup a keyboard simulation interface. */
  public abstract void configureSimKeyboard();

  /**************************
   *    Autonomous Setup    *
   **************************/

  /** Implemented by child classes to list all the commands we implement. */
  protected void addAutoChoices() {}

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
    return m_pathPlannerChooser.getSelected();
  }

  /** Run periodically by the ChaosRobot class */
  public abstract void periodic();

  /**
   * Attempts to update the pose estimator within the swerve drive object. Note that the SwerveDrive
   * may disregard pose updates if
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
