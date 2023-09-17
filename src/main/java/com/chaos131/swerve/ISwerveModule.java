// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.swerve;

import com.chaos131.pid.PIDFValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public interface ISwerveModule {

  public String getName();
  public void addCoachTabDashboardValues(ShuffleboardTab coachTab);
  public void periodic();

  public void driverModeInit();
  public void driveToPositionInit();

  public double getAbsoluteAngle();
  public double getAngleEncoderRatio();
  public double getVelocityEncoderRatio();
  public double getWheelCircumference();
  public double getRawAbsoluteAngle();

  public void setTarget(SwerveModuleState state);
  public void stop();
  public void setParkMode();

  public SwerveModuleState getModuleState();
  public Translation2d getTranslation();
  public SwerveModulePosition getPosition();

  public void UpdateVelocityPIDConstants(PIDFValue update);
  public void UpdateAnglePIDConstants(PIDFValue update);
  public void recalibrate();

  /**
   * This function takes in the current angle read by the encoder and a target angle for the robot to move to.
   * The target angle will be between -PI and PI, but this function will scale it up so it is an equivalent
   * angle that is closer to the current encoder angle. It will return this "optimized" angle to avoid
   * the wheels overspinning. 
   * @param currentModuleAngle_deg - The current swerve module's angle in degrees
   * @param targetAngle_deg - The target angle in degrees
   * @return The swerve module target angle 
   */
  public static double closestTarget(double currentModuleAngle_deg, double targetAngle_deg) {
    Rotation2d currentModuleAngle = Rotation2d.fromDegrees(currentModuleAngle_deg);
    Rotation2d targetAngle = Rotation2d.fromDegrees(targetAngle_deg);
    Rotation2d angleDifference = currentModuleAngle.minus(targetAngle);
    return currentModuleAngle_deg - angleDifference.getDegrees();
  } 


}
// "If you don’t agree to focus, you’re going to the business team." -Josh 2/21/2023