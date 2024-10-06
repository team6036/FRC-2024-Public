package com.peninsula.frc2024.util.control;

import com.peninsula.frc2024.config.SwerveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveOutputs {

  private final double[] outputsVoltage = new double[4];
  private final double[] outputsRadians = new double[4];
  private SwerveModuleState[] states = new SwerveModuleState[4];
  private SwerveModulePosition[] positions = new SwerveModulePosition[4];
  private Rotation2d rotation2d = new Rotation2d(0);
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

  private boolean idle = false;

  public SwerveOutputs() {
    for (int i = 0; i < 4; i++) {
      outputsVoltage[i] = 0;
      outputsRadians[i] = 0;
      positions[i] = new SwerveModulePosition();
    }
  }

  public SwerveOutputs(SwerveModuleState[] moduleStates) {
    setOutputs(moduleStates);
  }

  public SwerveOutputs(SwerveModuleState[] moduleStates, Rotation2d rotation2d) {
    for (int i = 0; i < 4; i++) {
      outputsVoltage[i] = 0;
      outputsRadians[i] = 0;
      positions[i] = new SwerveModulePosition();
    }
    setOutputs(moduleStates, rotation2d);
  }

  public SwerveOutputs(SwerveModuleState[] moduleStates, ChassisSpeeds chassisSpeeds) {
    for (int i = 0; i < 4; i++) {
      outputsVoltage[i] = 0;
      outputsRadians[i] = 0;
      positions[i] = new SwerveModulePosition();
    }
    setOutputs(moduleStates, chassisSpeeds);
  }

  public void setOutputs(SwerveModuleState[] moduleStates) {
    for (int i = 0; i < 4; i++) {
      outputsVoltage[i] =
          moduleStates[i].speedMetersPerSecond
              / SwerveConstants.Constants.Swerve.maxSpeed
              * SwerveConstants.kMaxVoltage;
      outputsRadians[i] = moduleStates[i].angle.getRadians();
    }
    for (int i = 0; i < 4; i++) {
      positions[i].distanceMeters += moduleStates[i].speedMetersPerSecond * 0.02;
      positions[i].angle = moduleStates[i].angle;
    }
    states = moduleStates;
  }

  public void setOutputs(SwerveModuleState[] moduleStates, Rotation2d rotation2d) {
    setOutputs(moduleStates);
    this.rotation2d = rotation2d;
  }

  public void setOutputs(SwerveModuleState[] moduleStates, ChassisSpeeds chassisSpeeds) {
    setOutputs(moduleStates);
    this.chassisSpeeds = chassisSpeeds;
  }

  public Rotation2d getRotation2d() {
    return rotation2d;
  }

  public SwerveModuleState[] getStates() {
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    return positions;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return chassisSpeeds;
  }

  public void setVoltages(double[] voltages) {
    System.arraycopy(voltages, 0, outputsVoltage, 0, 4);
  }

  public void setAngles(double[] radians) {
    System.arraycopy(radians, 0, outputsRadians, 0, 4);
  }

  public double[] steerAngles() {
    return outputsRadians;
  }

  public double[] voltages() {
    return outputsVoltage;
  }

  public boolean isIdle() {
    return idle;
  }

  public void setIdle(boolean idle) {
    this.idle = idle;
  }
}
