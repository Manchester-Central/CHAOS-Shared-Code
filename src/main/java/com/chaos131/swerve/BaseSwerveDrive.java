// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.swerve;

import com.chaos131.pid.PIDFValue;
import com.chaos131.pid.PIDTuner;
import com.chaos131.vision.VisionData;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.AutoLog;

/** The larger SwerveDrive subsystem that manages each swerve module. */
public class BaseSwerveDrive extends SubsystemBase {
  /** Tracks the state of the swerve system in AdvantageKit */
  @AutoLog
  public static class SwerveState {
    /** The velocity of the entire robot */
    public double velocity;

    /** The angle of the robot */
    public Rotation2d gyro;
  }

  /** List of swerve modules */
  protected List<BaseSwerveModule> m_swerveModules;

  /** Supplier that returns the current rotation of the robot, from a gyro */
  protected Supplier<Rotation2d> m_getRotation;

  /** Structure that converts between individual swerve modules, and the overall swerve system */
  protected SwerveDriveKinematics m_kinematics;

  /**
   * Structure that estimates the robot's location in the field using motor encoder values, and
   * updates from the camera system
   */
  protected SwerveDrivePoseEstimator m_odometry;

  /**
   * Determines if the entire swerve subsystem and pose estimator will accept updates from any
   * vision system
   */
  protected boolean m_acceptVisionUpdates;

  /**
   * Represents the robot's 2d location on the field, updated from the pose estimator in periodic()
   */
  protected Field2d m_field;

  /** Structure to approximate rotation while simulating the robot */
  protected Rotation2d m_simrotation = new Rotation2d();

  /** PID controller defining the location */
  protected PIDController m_XPid;

  /** PID controller defining the location */
  protected PIDController m_YPid;

  /**
   * PID Controller to manage the robot angle, since there is error stackup with many swerve modules
   * in motion
   */
  protected PIDController m_AngleDegreesPid;

  /** Tuner for the XPid */
  protected PIDTuner m_XPidTuner;

  /** Tuner for the YPid */
  protected PIDTuner m_YPidTuner;

  /** Tuner for the swerve and robot angle */
  protected PIDTuner m_AnglePidTuner;

  /** Manages the robot speed, to account for error in the floor quality */
  protected PIDTuner m_moduleVelocityPIDTuner;

  /** Manages the robot angle, to account for error in the field quality */
  protected PIDTuner m_moduleAnglePIDTuner;

  /** Default tolerance used by the X and Y location PID controller */
  protected double m_driveToTargetTolerance;

  /** Config structure for the overall swerve system */
  protected SwerveConfigs m_swerveConfigs;

  /**
   * Creates a new SwerveDrive subsystem
   *
   * @param swerveModules native array of SwerveDrive modules that make the robot move
   * @param swerveConfigs configuration structure that defines the broader swerve behavior
   * @param getRotation supplier that loads gyro data
   */
  public BaseSwerveDrive(
      BaseSwerveModule[] swerveModules,
      SwerveConfigs swerveConfigs,
      Supplier<Rotation2d> getRotation) {
    m_swerveModules = Arrays.asList(swerveModules);
    m_swerveConfigs = swerveConfigs;
    m_getRotation = getRotation;
    var isDebugMode = swerveConfigs.IsDebugMode();
    m_driveToTargetTolerance = swerveConfigs.defaultDriveToTargetTolerance();

    forAllModules(module -> module.setSimUpdatePeriod(m_swerveConfigs.updateFrequency_hz()));

    m_kinematics = new SwerveDriveKinematics(getModuleTranslations());
    Pose2d initialPoseMeters = new Pose2d(8, 4, Rotation2d.fromDegrees(0));
    m_odometry =
        new SwerveDrivePoseEstimator(
            m_kinematics, getGyroRotation(), getModulePositions(), initialPoseMeters);
    m_acceptVisionUpdates = true;
    resetPose(initialPoseMeters);
    m_field = new Field2d();
    SmartDashboard.putData("SwerveDrive", m_field);

    var translationPidValues = swerveConfigs.defaultTranslationPIDValues();
    m_XPid =
        new PIDController(translationPidValues.P, translationPidValues.I, translationPidValues.D);
    m_YPid =
        new PIDController(translationPidValues.P, translationPidValues.I, translationPidValues.D);
    m_XPidTuner = new PIDTuner("SwerveDrive/X_PID_Tuner", isDebugMode, m_XPid);
    m_YPidTuner = new PIDTuner("SwerveDrive/Y_PID_Tuner", isDebugMode, m_YPid);
    m_XPid.setTolerance(m_driveToTargetTolerance);
    m_YPid.setTolerance(m_driveToTargetTolerance);

    var rotationPidValues = swerveConfigs.defaultRotationPIDValues();
    m_AngleDegreesPid =
        new PIDController(rotationPidValues.P, rotationPidValues.I, rotationPidValues.D);
    m_AngleDegreesPid.enableContinuousInput(-180, 180);
    m_AngleDegreesPid.setTolerance(swerveConfigs.defaultRotationTolerance().getDegrees());
    m_AnglePidTuner = new PIDTuner("SwerveDrive/Angle_PID_Tuner", true, m_AngleDegreesPid);

    var moduleVelocityPIDF = m_swerveConfigs.defaultModuleVelocityPIDFValues();
    m_moduleVelocityPIDTuner =
        new PIDTuner(
            "SwerveDrive/ModuleVelocity_PID_Tuner",
            isDebugMode,
            moduleVelocityPIDF.P,
            moduleVelocityPIDF.I,
            moduleVelocityPIDF.D,
            moduleVelocityPIDF.F,
            this::updateVelocityPIDConstants);
    var moduleAnglePID = m_swerveConfigs.defaultModuleAnglePIDValues();
    m_moduleAnglePIDTuner =
        new PIDTuner(
            "SwerveDrive/ModuleAngle_PID_Tuner",
            isDebugMode,
            moduleAnglePID.P,
            moduleAnglePID.I,
            moduleAnglePID.D,
            this::updateAnglePIDConstants);
  }

  /**
   * Sets wether or not the swerve drive's odometry will accept vision updates, on a global level.
   *
   * @param state true = yes
   */
  public void setOdometryAcceptVisionData(boolean state) {
    m_acceptVisionUpdates = state;
  }

  /**
   * Applies some function across each swerve module, but doesn't return any values.
   *
   * @param lambdaFunction the function
   */
  public void forAllModules(Consumer<BaseSwerveModule> lambdaFunction) {
    m_swerveModules.forEach(lambdaFunction);
  }

  /**
   * Applies some function to the list of modules. Similar to forAllModules() but also returns some
   * values.
   *
   * @param <T> The return type
   * @param lambdaFunction the function used
   * @return a list of values, one for each module
   */
  public <T> List<T> mapModules(Function<BaseSwerveModule, T> lambdaFunction) {
    return m_swerveModules.stream().map(lambdaFunction).collect(Collectors.toList());
  }

  /**
   * I have no idea.
   *
   * @param coachTab I really don't.
   */
  public void addCoachTabDashboardValues(ShuffleboardTab coachTab) {
    forAllModules((module) -> module.addCoachTabDashboardValues(coachTab));
  }

  /** Sets up each module for driving, per the module implementation. */
  public void driverModeInit() {
    forAllModules((module) -> module.driverModeInit());
  }

  /** Sets up each module for driving to specific locations. */
  public void driveToPositionInit() {
    forAllModules((module) -> module.driveToPositionInit());
  }

  /**
   * Collects module position data for each module
   *
   * @return the swerve module position for each module
   */
  protected SwerveModulePosition[] getModulePositions() {
    return mapModules((module) -> module.getPosition()).toArray(SwerveModulePosition[]::new);
  }

  /**
   * Returns the list of module offsets for each module.
   *
   * @return that list
   */
  protected Translation2d[] getModuleTranslations() {
    return mapModules((module) -> module.getTranslation()).toArray(Translation2d[]::new);
  }

  /**
   * Returns the list of module states for each module.
   *
   * @return that list
   */
  protected SwerveModuleState[] getModuleStates() {
    return mapModules((module) -> module.getModuleState()).toArray(SwerveModuleState[]::new);
  }

  /**
   * Moves the robot according to the NORMALIZED speeds.
   *
   * @param chassisSpeeds the normalized speeds
   */
  public void move(ChassisSpeeds chassisSpeeds) {
    if (chassisSpeeds.vxMetersPerSecond == 0
        && chassisSpeeds.vyMetersPerSecond == 0
        && chassisSpeeds.omegaRadiansPerSecond == 0) {
      stop();
      return;
    }
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, m_swerveConfigs.maxRobotSpeed());
    for (var i = 0; i < states.length; i++) {
      m_swerveModules.get(i).setTarget(states[i]);
    }
  }

  /**
   * Puts the swerve drive system into "X" mode. This locks the robot in place by turning each
   * module parallel to the offset. Effectively no wheel can move without another wheel creating
   * significant drag.
   */
  public void setXMode() {
    forAllModules((module) -> module.setXMode());
  }

  /**
   * Sets each module to a specific state, this is typically a specific rotation without any
   * velocity.
   *
   * @param swerveModuleState The state to apply to each swerve module
   */
  public void forceUniformSwerveState(SwerveModuleState swerveModuleState) {
    forAllModules((module) -> module.setTarget(swerveModuleState));
  }

  /**
   * Moves the robot on the field while driving to a position automatically.
   *
   * @param xSpeed linear speed in the x direction
   * @param ySpeed linear speed in the y direction
   * @param omegaPercentSpeed angular speed in the CCW direction
   */
  public void moveFieldRelativeForPID(
      LinearVelocity xSpeed, LinearVelocity ySpeed, AngularVelocity omegaSpeed) {
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, getOdometryRotation());
    move(speeds);
  }

  /**
   * Moves the robot on the field from the perspective of the current driver station.
   *
   * @param xSpeed linear speed in the x direction
   * @param ySpeed linear speed in the y direction
   * @param omegaSpeed angular speed in the CCW direction
   */
  public void moveFieldRelative(
      LinearVelocity xSpeed, LinearVelocity ySpeed, AngularVelocity omegaSpeed) {
    ChassisSpeeds speeds;
    if (isDefaultAlliance()) {
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, getOdometryRotation());
    } else {
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed, omegaSpeed, getOdometryRotation().minus(new Rotation2d(Math.PI)));
    }
    move(speeds);
  }

  /**
   * Moves the robot on the field while driving to a position automatically.
   *
   * @param xSpeed linear speed in the x direction
   * @param ySpeed linear speed in the y direction
   * @param angle the angle on the field to target
   * @param magnitude how quickly to target the angle [0, 1.0]
   */
  public void moveFieldRelativeAngle(
      LinearVelocity xSpeed, LinearVelocity ySpeed, Rotation2d angle, double magnitude) {
    double omega = 0;
    ChassisSpeeds speeds;
    if (Math.abs(magnitude) >= 0.2) {
      omega =
          m_AngleDegreesPid.calculate(getOdometryRotation().getDegrees(), angle.getDegrees())
              * magnitude;
    }
    if (isDefaultAlliance()) {
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed, Units.RadiansPerSecond.of(omega), getOdometryRotation());
    } else {
      // Rotate the direction if we're on the non default side (if 0,0 is on the blue side and we're
      // red, we need to rotate our perspective)
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed,
              ySpeed,
              Units.RadiansPerSecond.of(omega),
              getOdometryRotation().minus(new Rotation2d(Math.PI)));
    }
    move(speeds);
  }

  /**
   * Moves robot relative mode while maintaining field relative angle.
   *
   * @param xSpeed linear speed in the x direction
   * @param ySpeed linear speed in the y direction
   * @param angle the angle on the field to target
   * @param magnitude how quickly to target the angle [0, 1.0]
   */
  public void moveRobotRelativeAngle(
      LinearVelocity xSpeed, LinearVelocity ySpeed, Rotation2d angle, double magnitude) {
    double omega = 0;
    if (Math.abs(magnitude) >= 0.2) {
      omega =
          m_AngleDegreesPid.calculate(getOdometryRotation().getDegrees(), angle.getDegrees())
              * magnitude;
    }

    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, Units.RadiansPerSecond.of(omega));
    move(speeds);
  }

  /**
   * Moves the robot relative to itself.
   *
   * @param xSpeed linear speed in the x direction
   * @param ySpeed linear speed in the y direction
   * @param omegaSpeed angular speed in the CCW direction
   */
  public void moveRobotRelative(
      LinearVelocity xSpeed, LinearVelocity ySpeed, AngularVelocity omegaSpeed) {
    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed);
    move(speeds);
  }

  /**
   * Checks if we're currently running as what we consider the default alliance (2023 we went with
   * red - 2024 we are going with blue)
   *
   * @return true if we are the default, per community documentation blue is the default from 2024
   *     onward
   */
  protected boolean isDefaultAlliance() {
    return DriverStation.getAlliance().orElse(m_swerveConfigs.defaultAlliance())
        == m_swerveConfigs.defaultAlliance();
  }

  /** Resets the PID controllers for X, Y, and Rotation. */
  public void resetPids() {
    m_XPid.reset();
    m_YPid.reset();
    m_AngleDegreesPid.reset();
    setDriveTranslationTolerance(m_swerveConfigs.defaultDriveToTargetTolerance());
  }

  /**
   * @return true if the odometry thinks the robot is close enough to the target location.
   */
  public boolean atTarget() {
    return atTarget(m_driveToTargetTolerance);
  }

  /**
   * TODO: Fix me to use the pose estimator!
   *
   * @param tolerance floor distance in field units to be "close enough"
   * @return true if in range
   */
  public boolean atTarget(double tolerance) {
    boolean isXTolerable = Math.abs(getPose().getX() - m_XPid.getSetpoint()) <= tolerance;
    boolean isYTolerable = Math.abs(getPose().getY() - m_YPid.getSetpoint()) <= tolerance;
    return isXTolerable && isYTolerable && m_AngleDegreesPid.atSetpoint();
  }

  /**
   * Sets the location for the swerve system to aim for.
   *
   * @param x - Destination x coordinate, uses the choosen field coordinate system
   * @param y - Destination y coordinate, uses the choosen field coordinate system
   * @param angle - The angle or heading of the robot at the destination
   */
  public void setTarget(double x, double y, Rotation2d angle) {
    m_XPid.setSetpoint(x);
    m_YPid.setSetpoint(y);
    m_AngleDegreesPid.setSetpoint(angle.getDegrees());
  }

  /**
   * Sets the location for the swerve system to aim for.
   *
   * @param loc the target location as understood by the PIDs
   * @param angle the desired location as understood by the PIDs
   */
  public void setTarget(Transform2d loc, Rotation2d angle) {
    m_XPid.setSetpoint(loc.getX());
    m_YPid.setSetpoint(loc.getY());
    m_AngleDegreesPid.setSetpoint(angle.getDegrees());
  }

  /**
   * Sets the location for the swerve system to aim for.
   *
   * @param pose - A Pose2d representing the target location and orientation
   */
  public void setTarget(Pose2d pose) {
    m_XPid.setSetpoint(pose.getTranslation().getX());
    m_YPid.setSetpoint(pose.getTranslation().getY());
    m_AngleDegreesPid.setSetpoint(pose.getRotation().getDegrees());
  }

  /**
   * Moves the robot to the target location and orientation at a percentage of the total speed. This
   * relies on the moveFieldRelativeForPID functionality.
   *
   * @param maxTranslationSpeedPercent a value in the range of [0, 1]
   */
  public void moveToTarget(double maxTranslationSpeedPercent) {
    Pose2d pose = getPose();

    Translation2d difference =
        pose.getTranslation().minus(new Translation2d(m_XPid.getSetpoint(), m_YPid.getSetpoint()));

    var normalizedDifference = difference.div(difference.getNorm());

    double x =
        MathUtil.clamp(
            m_XPid.calculate(pose.getX()),
            -(maxTranslationSpeedPercent * normalizedDifference.getX()),
            (maxTranslationSpeedPercent * normalizedDifference.getX()));
    double y =
        MathUtil.clamp(
            m_YPid.calculate(pose.getY()),
            -(maxTranslationSpeedPercent * normalizedDifference.getY()),
            (maxTranslationSpeedPercent * normalizedDifference.getY()));
    double angle = m_AngleDegreesPid.calculate(pose.getRotation().getDegrees());
    moveFieldRelativeForPID(
        Units.MetersPerSecond.of(x), Units.MetersPerSecond.of(y), Units.RadiansPerSecond.of(angle));
  }

  /**
   * @return the robot's rotation determined from the gyro (or simulated rotation)
   */
  public Rotation2d getGyroRotation() {
    if (RobotBase.isSimulation()) {
      return m_simrotation;
    }
    return m_getRotation.get();
  }

  /**
   * @return the floor pose of the robot from the pose estimator
   */
  public Pose2d getPose() {
    synchronized (m_odometry) {
      return m_odometry.getEstimatedPosition();
    }
  }

  /**
   * @return the rotation of the robot from the pose estimator
   */
  public Rotation2d getOdometryRotation() {
    return getPose().getRotation();
  }

  /** Tells each swerve module to run their recalibration method */
  public void recalibrateModules() {
    forAllModules((module) -> module.recalibrate());
  }

  /** Tells each swerve module to run their stop method */
  public void stop() {
    forAllModules((module) -> module.stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotBase.isSimulation()) {
      ChassisSpeeds speeds = m_kinematics.toChassisSpeeds(getModuleStates());
      double radians = speeds.omegaRadiansPerSecond / m_swerveConfigs.updateFrequency_hz();
      m_simrotation = m_simrotation.plus(Rotation2d.fromRadians(radians));
    }
    final Pose2d robotPose;
    synchronized (m_odometry) {
      robotPose = m_odometry.update(getGyroRotation(), getModulePositions());
    }

    m_field.setRobotPose(robotPose);
    forAllModules((module) -> updateModuleOnField(module, robotPose));
    m_XPidTuner.tune();
    m_YPidTuner.tune();
    m_AnglePidTuner.tune();
    m_moduleVelocityPIDTuner.tune();
    m_moduleAnglePIDTuner.tune();
    forAllModules((module) -> module.periodic());
  }

  /**
   * Resets the robot's position on the field.
   *
   * <p>The gyroscope angle does not need to be reset in the user's robot code. The library
   * automatically takes care of offsetting the gyro angle.
   *
   * @param targetPose the pose to set to
   */
  public void resetPose(Pose2d targetPose) {
    if (RobotBase.isSimulation()) {
      m_simrotation = targetPose.getRotation();
    }
    synchronized (m_odometry) {
      m_odometry.resetPosition(getGyroRotation(), getModulePositions(), targetPose);
    }
  }

  /**
   * Resets just the heading, not the entire pose. Passes off to resetPose().
   *
   * @param targetHeading the angle to use
   */
  public void resetHeading(Rotation2d targetHeading) {
    var currentPose = getPose();
    var updatedPose = new Pose2d(currentPose.getX(), currentPose.getY(), targetHeading);
    resetPose(updatedPose);
  }

  /**
   * Updates a specific swervemodule based on the robot pose in the field.
   *
   * @param swerveModule the module
   * @param robotPose the current robot pose
   */
  public void updateModuleOnField(BaseSwerveModule swerveModule, Pose2d robotPose) {
    if (!m_swerveConfigs.IsDebugMode()) {
      return;
    }
    Transform2d transform =
        new Transform2d(
            swerveModule.getTranslation().times(5), swerveModule.getModuleState().angle);
    Pose2d swerveModulePose = robotPose.transformBy(transform);
    m_field.getObject(swerveModule.getName()).setPose(swerveModulePose);
  }

  /**
   * Applies the PID values to every module in the swerve system
   *
   * @param update the 4 PIDF values
   */
  protected void updateVelocityPIDConstants(PIDFValue update) {
    forAllModules((module) -> module.updateVelocityPIDConstants(update));
  }

  /**
   * Applies the values to every module in the swerve system
   *
   * @param update the 4 PIDF vlues
   */
  protected void updateAnglePIDConstants(PIDFValue update) {
    forAllModules((module) -> module.updateAnglePIDConstants(update));
  }

  /**
   * Applies the translation tolerance to atTarget() and the individual XPid and YPid controllers
   *
   * @param tolerance value in field units
   */
  public void setDriveTranslationTolerance(double tolerance) {
    m_driveToTargetTolerance = tolerance;
    m_XPid.setTolerance(m_driveToTargetTolerance);
    m_YPid.setTolerance(m_driveToTargetTolerance);
  }

  /**
   * @return robot's movement speed in field units per second
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * @return the robot translation speed (in any direction) in meters per second
   */
  public LinearVelocity getRobotSpeed() {
    var robotSpeeds = getRobotRelativeSpeeds();
    var xMetersPerSecond = robotSpeeds.vxMetersPerSecond;
    var yMetersPerSecond = robotSpeeds.vyMetersPerSecond;
    return Units.MetersPerSecond.of(
        Math.sqrt(Math.pow(xMetersPerSecond, 2) + Math.pow(yMetersPerSecond, 2)));
  }

  /**
   * @return double - The angular velocity of the robot in radians per second
   */
  public AngularVelocity getRobotRotationSpeed() {
    return Units.RadiansPerSecond.of(getRobotRelativeSpeeds().omegaRadiansPerSecond);
  }

  /**
   * Sets the module PID targets based on the chassis speeds, for use with path planner
   *
   * @param chassisSpeeds those chassis speeds
   */
  public void pathPlannerRobotRelative(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    // SwerveDriveKinematics.desaturateWheelSpeeds(states, m_swerveConfigs.maxRobotSpeed_mps());
    for (var i = 0; i < states.length; i++) {
      m_swerveModules.get(i).setTarget(states[i]);
    }
  }

  /**
   * Translates the robot's current pose with forward moving in the direction of the current angle
   * and left moving orthogonal to the current angle
   *
   * @param robotForwardMeters the meters to move forward (or backwards if negative) in relation to
   *     the current pose and direction
   * @param robotLeftMeters the meters to move left (or right if negative) in relation to the
   *     current pose and direction
   * @return the new pose
   */
  public Pose2d getTranslatedPose(Distance robotForwardMeters, Distance robotLeftMeters) {
    return getPose()
        .transformBy(
            new Transform2d(robotForwardMeters, robotLeftMeters, Rotation2d.fromDegrees(0)));
  }

  /**
   * The preferred way of adding supplemental pose information into the pose estimator.
   *
   * <p>Increase the values of the deviation to decrease the confidence in those values.
   *
   * <p>This method is thread safe, as the wpilib overloaded addVisionMeasurement() is _not_ thread
   * safe.
   *
   * @param data data from the camera system
   */
  public void addVisionMeasurement(VisionData data) {
    if (!m_acceptVisionUpdates) return;
    synchronized (m_odometry) {
      m_odometry.addVisionMeasurement(
          data.getPose2d(), data.getTimestampSeconds(), data.getDeviationMatrix());
    }
  }
}
