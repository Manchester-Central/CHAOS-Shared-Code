// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.swerve;

import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import com.chaos131.logging.LogManager;
import com.chaos131.pid.PIDFValue;
import com.chaos131.pid.PIDTuner;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BaseSwerveDrive extends SubsystemBase {

  public static double TranslationSpeedModifier = 1.0;
  public static double RotationSpeedModifier = 1.0;

  private List<BaseSwerveModule> m_swerveModules;
  private Supplier<Rotation2d> m_getRotation;

  private SwerveDriveKinematics m_kinematics;
  private SwerveDrivePoseEstimator m_odometry;
  private Field2d m_field;
  private Rotation2d m_simrotation = new Rotation2d();

  private PIDController m_XPid;
  private PIDController m_YPid;
  private PIDController m_AngleDegreesPid;
  private PIDTuner m_XPidTuner;
  private PIDTuner m_YPidTuner;
  private PIDTuner m_AnglePidTuner;
  private PIDTuner m_moduleVelocityPIDTuner;
  private PIDTuner m_moduleAnglePIDTuner;
  private double m_driveToTargetTolerance;
  private LogManager m_logger = LogManager.getInstance();
  private SwerveConfigs m_swerveConfigs;

  /** Creates a new SwerveDrive. */
  public BaseSwerveDrive(BaseSwerveModule[] swerveModules, SwerveConfigs swerveConfigs, Supplier<Rotation2d> getRotation) {
    m_swerveModules = Arrays.asList(swerveModules);
    m_swerveConfigs = swerveConfigs;
    m_getRotation = getRotation;
    var isDebugMode = swerveConfigs.IsDebugMode();
    m_driveToTargetTolerance = swerveConfigs.defaultDriveToTargetTolerance();

    m_kinematics = new SwerveDriveKinematics(
        getModuleTranslations());
    Pose2d initialPoseMeters = new Pose2d(8, 4, Rotation2d.fromDegrees(0));
    m_odometry = new SwerveDrivePoseEstimator(
        m_kinematics, getGyroRotation(),
        getModulePositions(),
        initialPoseMeters);
    resetPose(initialPoseMeters);
    m_field = new Field2d();
    SmartDashboard.putData("SwerveDrive", m_field);

    var translationPidValues = swerveConfigs.defaultTranslationPIDValues();
    m_XPid = new PIDController(translationPidValues.P, translationPidValues.I, translationPidValues.D);
    m_YPid = new PIDController(translationPidValues.P, translationPidValues.I, translationPidValues.D);
    m_XPidTuner = new PIDTuner("SwerveDrive/X_PID_Tuner", isDebugMode, m_XPid);
    m_YPidTuner = new PIDTuner("SwerveDrive/Y_PID_Tuner", isDebugMode, m_YPid);
    m_XPid.setTolerance(m_driveToTargetTolerance);
    m_YPid.setTolerance(m_driveToTargetTolerance);

    var rotationPidValues = swerveConfigs.defaultTranslationPIDValues();
    m_AngleDegreesPid = new PIDController(rotationPidValues.P, rotationPidValues.I, rotationPidValues.D);
    m_AngleDegreesPid.enableContinuousInput(-180, 180);
    m_AngleDegreesPid.setTolerance(swerveConfigs.defaultRotationTolerance().getDegrees());
    m_AnglePidTuner = new PIDTuner("SwerveDrive/Angle_PID_Tuner", true, m_AngleDegreesPid);

    var moduleVelocityPIDF = m_swerveConfigs.defaultModuleVelocityPIDFValues();
    m_moduleVelocityPIDTuner = new PIDTuner("Swerve/ModuleVelocity_PID_Tuner", isDebugMode, moduleVelocityPIDF.P, moduleVelocityPIDF.I, moduleVelocityPIDF.D, moduleVelocityPIDF.F, this::updateVelocityPIDConstants);
    var moduleAnglePID = m_swerveConfigs.defaultModuleAnglePIDValues();
    m_moduleAnglePIDTuner = new PIDTuner("Swerve/ModuleAngle_PID_Tuner", isDebugMode, moduleAnglePID.P, moduleAnglePID.I, moduleAnglePID.D, this::updateAnglePIDConstants);

    forAllModules(module -> module.setSimUpdateFrequency_hz(m_swerveConfigs.updateFrequency_hz()));

    m_logger.addNumber("SwerveDrive/X_m", isDebugMode, () -> getPose().getX());
    m_logger.addNumber("SwerveDrive/Y_m", isDebugMode, () -> getPose().getY());
    m_logger.addNumber("SwerveDrive/Rotation_deg", isDebugMode, () -> getOdometryRotation().getDegrees());
    m_logger.addNumber("SwerveDrive/Gyro_angle_deg", isDebugMode, () -> getGyroRotation().getDegrees());

  }

  public void forAllModules(Consumer<BaseSwerveModule> lambdaFunction) {
    m_swerveModules.forEach(lambdaFunction);
  }

  public <T> List<T> mapModules(Function<BaseSwerveModule, T> lambdaFunction) {
    return m_swerveModules.stream().map(lambdaFunction).toList();
  }
    
  public void addCoachTabDashboardValues(ShuffleboardTab coachTab) {
    forAllModules((module) -> module.addCoachTabDashboardValues(coachTab));
  }

  public void driverModeInit() {
    forAllModules((module) -> module.driverModeInit());
  }

  public void driveToPositionInit() {
    forAllModules((module) -> module.driveToPositionInit());
  }

  private SwerveModulePosition[] getModulePositions() {
    return mapModules((module) -> module.getPosition()).toArray(SwerveModulePosition[] ::new);
  }

  private Translation2d[] getModuleTranslations() {
    return mapModules((module) -> module.getTranslation()).toArray(Translation2d[] ::new);
  }

  private SwerveModuleState[] getModuleStates() {
    return mapModules((module) -> module.getModuleState()).toArray(SwerveModuleState[] ::new);
  }

  public void move(ChassisSpeeds chassisSpeeds) {
    chassisSpeeds.vxMetersPerSecond = MathUtil.clamp(chassisSpeeds.vxMetersPerSecond, -1, 1) * m_swerveConfigs.maxRobotSpeed_mps() * TranslationSpeedModifier;
    chassisSpeeds.vyMetersPerSecond = MathUtil.clamp(chassisSpeeds.vyMetersPerSecond, -1, 1) * m_swerveConfigs.maxRobotSpeed_mps() * TranslationSpeedModifier;
    chassisSpeeds.omegaRadiansPerSecond = MathUtil.clamp(chassisSpeeds.omegaRadiansPerSecond, -1, 1) * m_swerveConfigs.maxRobotRotation_radps() * RotationSpeedModifier;
    if (chassisSpeeds.vxMetersPerSecond == 0 && chassisSpeeds.vyMetersPerSecond == 0 && chassisSpeeds.omegaRadiansPerSecond == 0) {
      stop();
      return;
    }
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, m_swerveConfigs.maxRobotSpeed_mps());
    for (var i = 0; i < states.length; i++) {
      m_swerveModules.get(i).setTarget(states[i]);
    }
  }

  public void setXMode() {
    forAllModules((module) -> module.setXMode());
  }

  public void debug_setSwerveModule(SwerveModuleState swerveModuleState) {
    forAllModules((module) -> module.setTarget(swerveModuleState));
  }

  public void moveFieldRelativeForPID(double xMetersPerSecond, double yMetersPerSecond, double omegaRadianPerSecond){
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, omegaRadianPerSecond, getOdometryRotation());
    move(speeds);
  }

  public void moveFieldRelative(double xMetersPerSecond, double yMetersPerSecond, double omegaRadianPerSecond){
    ChassisSpeeds speeds;
    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, omegaRadianPerSecond, getOdometryRotation().minus(new Rotation2d(Math.PI)));
    } else {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, omegaRadianPerSecond, getOdometryRotation());
    }
    move(speeds);
  }

  public void moveFieldRelativeAngle(double xMetersPerSecond, double yMetersPerSecond, Rotation2d angle, double magnitude){
    double omega = 0;
    ChassisSpeeds speeds;
    if (Math.abs(magnitude) >= 0.2) {
      omega = m_AngleDegreesPid.calculate(getOdometryRotation().getDegrees(), angle.getDegrees()) * magnitude;
    }
    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, omega, getOdometryRotation().minus(new Rotation2d(Math.PI)));
    } else {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, omega, getOdometryRotation());
    }
    move(speeds);
  } 

  public void moveRobotRelative(double xForwardSpeedMetersPerSecond, double ySidewaySpeedMetersPerSecond,
      double omegaRadianPerSecond) {
    ChassisSpeeds speeds = new ChassisSpeeds(xForwardSpeedMetersPerSecond, ySidewaySpeedMetersPerSecond,
        omegaRadianPerSecond);
    move(speeds);
  }

  public void resetPids() {
    m_XPid.reset();
    m_YPid.reset();
    m_AngleDegreesPid.reset();
    setDriveTranslationTolerance(m_swerveConfigs.defaultDriveToTargetTolerance());
  }

  public boolean atTarget() {
    boolean isXTolerable = Math.abs(getPose().getX() - m_XPid.getSetpoint()) <= m_driveToTargetTolerance;
    boolean isYTolerable = Math.abs(getPose().getY() - m_YPid.getSetpoint()) <= m_driveToTargetTolerance;
    return isXTolerable && isYTolerable && m_AngleDegreesPid.atSetpoint();

  }

  public void setTarget(double x, double y, Rotation2d angle) {
    m_XPid.setSetpoint(x);
    m_YPid.setSetpoint(y);
    m_AngleDegreesPid.setSetpoint(angle.getDegrees());
  }

  public void moveToTarget(double maxTranslationSpeedPercent) {
    Pose2d pose = getPose();
    double x = MathUtil.clamp(m_XPid.calculate(pose.getX()), -maxTranslationSpeedPercent, maxTranslationSpeedPercent);
    double y = MathUtil.clamp(m_YPid.calculate(pose.getY()), -maxTranslationSpeedPercent, maxTranslationSpeedPercent);
    double angle = m_AngleDegreesPid.calculate(pose.getRotation().getDegrees());
    moveFieldRelativeForPID(x, y, angle);
  }

  public Rotation2d getGyroRotation() {
    if (RobotBase.isSimulation()) {
      return m_simrotation;
    }
    return m_getRotation.get().times(-1);
  }

  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  public Rotation2d getOdometryRotation() {
    return getPose().getRotation();
  }

  public void recalibrateModules(){
    forAllModules((module) -> module.recalibrate());
  }

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
    Pose2d robotPose = m_odometry.update(getGyroRotation(), getModulePositions());
    m_field.setRobotPose(robotPose);
    forAllModules((module) -> updateModuleOnField(module, robotPose));
    m_XPidTuner.tune();
    m_YPidTuner.tune();
    m_AnglePidTuner.tune();
    m_moduleVelocityPIDTuner.tune();
    m_moduleAnglePIDTuner.tune();
    forAllModules((module) -> module.periodic());
  }

  public void resetPose(Pose2d targetPose){
    if (RobotBase.isSimulation()) {
      m_simrotation = targetPose.getRotation();
    }
    m_odometry.resetPosition(getGyroRotation(), getModulePositions(), targetPose);
  }

  public void resetHeading(Rotation2d targetHeading) {
    var currentPose = getPose();
    var updatedPose = new Pose2d(currentPose.getX(), currentPose.getY(), targetHeading);
    resetPose(updatedPose);
  }

  public void updateModuleOnField(BaseSwerveModule swerveModule, Pose2d robotPose) {
    if (!m_swerveConfigs.IsDebugMode()) {
      return;
    }
    Transform2d transform = new Transform2d(swerveModule.getTranslation().times(5), swerveModule.getModuleState().angle);
    Pose2d swerveModulePose = robotPose.transformBy(transform);
    m_field.getObject(swerveModule.getName()).setPose(swerveModulePose);
  }

  private void updateVelocityPIDConstants(PIDFValue update) {
    forAllModules((module) -> module.UpdateVelocityPIDConstants(update));
  }

  private void updateAnglePIDConstants(PIDFValue update) {
    forAllModules((module) -> module.UpdateAnglePIDConstants(update));
  }

  public void setDriveTranslationTolerance(double tolerance) {
    m_driveToTargetTolerance = tolerance;
  }

  public void addVisionMeasurement(int xMeters, int yMeters) {
    addVisionMeasurement(new Pose2d(xMeters, yMeters, getOdometryRotation()));
  }

  public void addVisionMeasurement(Pose2d measuredPose) {
    if (measuredPose != null) {
      m_odometry.addVisionMeasurement(measuredPose, Timer.getFPGATimestamp());
    }
  }
}
// "I love polyester." -Kenny