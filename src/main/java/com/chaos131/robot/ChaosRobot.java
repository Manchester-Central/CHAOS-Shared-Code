package com.chaos131.robot;

import com.chaos131.SharedCodeBuildConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Optional;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * A ChaosRobot architecture that bundles together common libraries like AdvantageKit, PathPlanner,
 * and common features like mode timers.
 */
public class ChaosRobot extends LoggedRobot {

  /** A container for the majority of the robot code, containing all of the subsystems */
  protected ChaosRobotContainer m_robotContainer;

  /** A command choosen for Autonomous Mode */
  private Command m_autoCmd;

  /**********************
   *    System Modes    *
   **********************/

  /** The mode of the robot, typically this is real mode. */
  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY,

    /** Running in demo mode. */
    DEMO
  }

  /** The mode of the robot, typically this is real mode. */
  private Mode m_mode;

  /**
   * @param m the mode to set for the robot
   */
  protected void setRobotMode(Mode m) {
    m_mode = m;
  }

  /***********************
   *    System Timers    *
   ***********************/

  /**
   * Timer to do any post-disable functions and state changes, in 2024 this was to loosen the motors
   * on the lift
   */
  private double m_disabledCleanupSeconds = 10;

  /**
   * @param val time in seconds for the disable cleanup to happen
   */
  protected void setDisabledCleanupTimer(double val) {
    m_disabledCleanupSeconds = val;
  }

  /** Timer that starts when entering any phase */
  private Optional<Double> m_timerStart = Optional.empty();

  /** global timer that captures the time the robot booted up */
  private double m_timerGlobal;

  /**
   * @return time since the robot started up
   */
  protected double getTotalRunningTime() {
    return Timer.getFPGATimestamp() - m_timerGlobal;
  }

  /** Starts the timer for TeleOp mode, overwriting any timer that may already be there. */
  protected void timerStart() {
    m_timerStart = Optional.of(Timer.getFPGATimestamp());
  }

  /**
   * Gets the running time for TeleOp mode.
   *
   * @return Time in seconds (Empty if the timer wasn't started)
   */
  protected Optional<Double> timerRunningTime() {
    if (m_timerStart.isEmpty()) return m_timerStart;
    double running_time = Timer.getFPGATimestamp() - m_timerStart.get();
    return Optional.of(running_time);
  }

  /**
   * Ends the TeleOp timer, and returns the elapsed time if the timer was started.
   *
   * @return Time in seconds (Empty if the timer wasn't started)
   */
  protected Optional<Double> timerEnd() {
    var state = timerRunningTime();
    m_timerStart = Optional.empty();
    return state;
  }

  /**************************
   *    Enabled/Disabled    *
   **************************/

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code. Probably shouldn't be completely overrode, but instead call super() at the
   * start of child class versions.
   */
  @Override
  public void robotInit() {
    m_timerGlobal = Timer.getFPGATimestamp();

    switch (m_mode) {
      case REAL:
        setupLoggerRealMode();
        break;

      case SIM:
        setupLoggerSimMode();
        break;

      case REPLAY:
        setupLoggerReplayMode();
        break;

      case DEMO:
      default:
        break;
    }

    Logger.recordMetadata("SharedHash", SharedCodeBuildConstants.GIT_SHA);
    Logger.recordMetadata("SharedCommitDate", SharedCodeBuildConstants.GIT_DATE);
    Logger.recordMetadata("SharedDirty", (SharedCodeBuildConstants.DIRTY == 1) ? "Yes" : "No");
    Logger.recordMetadata("SharedBuildDate", SharedCodeBuildConstants.BUILD_DATE);

    setupRobot();

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the
    // "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.
  }

  /**
   * Highly encouraged to replace this method with any robot specific setups. Things like
   * BuildConstants, voltage logging, etc.
   */
  protected void setupRobot() {
    Logger.recordMetadata("ProjectName", "Another FRC Project"); // Set a metadata value
    // Something like...
    // Logger.recordMetadata("RobotHash", BuildConstants.GIT_SHA);
    // Logger.recordMetadata("RobotBranch", BuildConstants.GIT_BRANCH);
    // Logger.recordMetadata("RobotDirty", (BuildConstants.DIRTY == 1) ? "Yes" : "No");
  }

  /**
   * Highly encouraged to override this method with robot specific setups, and to call super() at
   * the start of the method.
   *
   * <p>Sets up the...
   *
   * <p>File writer
   *
   * <p>NetworkTables
   */
  protected void setupLoggerRealMode() {
    Logger.addDataReceiver(new WPILOGWriter());
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
  }

  /** Sets up a simulation mode, called during robotInit(). */
  protected void setupLoggerSimMode() {
    // Running a physics simulator, log to NT
    Logger.addDataReceiver(new NT4Publisher());
  }

  /**
   * Sets up Replay Mode, called during robotInit(). Unlikely to ever need to replace this method.
   */
  protected void setupLoggerReplayMode() {
    // Replaying a log, set up replay source
    setUseTiming(false); // Run as fast as possible
    String logPath = LogFileUtil.findReplayLog();
    Logger.setReplaySource(new WPILOGReader(logPath));
    Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"), 0.01));
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.periodic();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    timerStart();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    var running_time = timerRunningTime();
    if (running_time.isPresent()) {
      if (m_disabledCleanupSeconds < running_time.get()) {
        disabledCleanup();
        timerEnd();
      }
    }
  }

  /**
   * Highly encouraged to replace this method with state changes designed to happen after the robot
   * is disabled.
   *
   * <p>This could be disabling specific motors, changing from locked to coast, or some other SAFE
   * behavior a robot should exhibit while on the field and potentially near people.
   */
  public void disabledCleanup() {
    Logger.recordOutput("Disabled Cleanup Running...");
  }

  /** This autonomous runs the autonomous command selected by your RobotContainer class. */
  @Override
  public void autonomousInit() {
    timerStart();
    var m_autoCmd = m_robotContainer.getAutonomousCommand();
    Logger.recordOutput("Received Auto Cmd", m_autoCmd.getName());
    if (m_autoCmd != null) {
      m_autoCmd.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    timerStart();
    if (m_autoCmd != null) {
      m_autoCmd.cancel();
      m_autoCmd = null;
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    timerStart();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    timerStart();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
