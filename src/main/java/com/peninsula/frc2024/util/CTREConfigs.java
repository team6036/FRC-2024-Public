package com.peninsula.frc2024.util;

// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix6.configs.*;
import com.peninsula.frc2024.config.SwerveConstants;

public final class CTREConfigs {
  public static TalonFXConfiguration swerveDriveFXConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    TorqueCurrentConfigs torqueCurrentConfigs = new TorqueCurrentConfigs();
    torqueCurrentConfigs.PeakForwardTorqueCurrent =
        SwerveConstants.Constants.Swerve.drivePeakCurrentLimit;
    torqueCurrentConfigs.PeakReverseTorqueCurrent =
        -SwerveConstants.Constants.Swerve.drivePeakCurrentLimit;
    config.TorqueCurrent = torqueCurrentConfigs;

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = SwerveConstants.Constants.Swerve.driveKP;
    slot0.kI = SwerveConstants.Constants.Swerve.driveKI;
    slot0.kD = SwerveConstants.Constants.Swerve.driveKD;
    slot0.kV = SwerveConstants.Constants.Swerve.driveKF;
    config.Slot0 = slot0;

    //    OpenLoopRampsConfigs openLoop = new OpenLoopRampsConfigs();
    //    openLoop.DutyCycleOpenLoopRampPeriod = SwerveConstants.Constants.Swerve.openLoopRamp;
    //    config.OpenLoopRamps = openLoop;
    ClosedLoopRampsConfigs closeLoop = new ClosedLoopRampsConfigs();
    closeLoop.DutyCycleClosedLoopRampPeriod = 0;
    closeLoop.TorqueClosedLoopRampPeriod = 0;
    closeLoop.VoltageClosedLoopRampPeriod = 0;
    config.ClosedLoopRamps = closeLoop;
    return config;
  }

  public static CANcoderConfiguration swerveCancoderConfig(double angleOffset) {
    CANcoderConfiguration CANconfig = new CANcoderConfiguration();
    MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();

    magnetSensorConfigs.MagnetOffset = angleOffset; // in rotations
    CANconfig.MagnetSensor = magnetSensorConfigs;

    return CANconfig;
  }
}
