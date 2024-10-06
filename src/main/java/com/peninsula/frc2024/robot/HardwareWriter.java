package com.peninsula.frc2024.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.peninsula.frc2024.config.*;
import com.peninsula.frc2024.logging.Logger;
import com.peninsula.frc2024.subsystems.*;
import com.peninsula.frc2024.util.config.TalonFXFactory;
import com.peninsula.frc2024.vision.PerceptionConfigurator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Set;

public class HardwareWriter {
  static PerceptionConfigurator configurator = new PerceptionConfigurator();

  void configureHardware(Set<SubsystemBase> enabledSubsystems) {
    if (enabledSubsystems.contains(Swerve.getInstance())) configureSwerveHardware();
    if (enabledSubsystems.contains(Shooter.getInstance())) configureShooterHardware();
    if (enabledSubsystems.contains(Intake.getInstance())) configureIntakeHardware();
    if (enabledSubsystems.contains(Arm.getInstance())) configureArmHardware();
    if (enabledSubsystems.contains(Turret.getInstance())) configureTurretHardware();
    if (enabledSubsystems.contains(Lighting.getInstance())) configureLightingHardware();
  }

  private void configureLightingHardware() {
    var hardware = HardwareAdapter.LightingHardware.getInstance();
    hardware.candle.animate(LightingConstants.rainbow);
  }

  private void configureTurretHardware() {
    var hardware = HardwareAdapter.TurretHardware.getInstance();

    TalonFXConfiguration configuration = new TalonFXConfiguration();

    /* Control gains/limits */
    configuration.Slot0.kP = TurretConstants.kP;
    configuration.Slot0.kI = TurretConstants.kI;
    configuration.Slot0.kD = TurretConstants.kD;
    configuration.Slot0.kV = TurretConstants.kF;
    configuration.Slot0.kS = TurretConstants.kS;

    configuration.MotionMagic.MotionMagicAcceleration = TurretConstants.mmAcceleration;
    configuration.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.mmCruiseVelocity;

    /* Inverted */
    configuration.MotorOutput.Inverted =
        TurretConstants.motorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    //    configuration.Feedback.FeedbackRemoteSensorID = hardware.absoluteEncoder.getDeviceID();
    configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    //    configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    /* Soft limits */
    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        TurretConstants.maxDeviationRotations * TurretConstants.gearRatio;
    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = TurretConstants.enableSoftLimits;

    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        TurretConstants.maxDeviationRotations * TurretConstants.gearRatio;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = TurretConstants.enableSoftLimits;

    configuration.CurrentLimits.SupplyCurrentLimit = 100;
    configuration.CurrentLimits.SupplyCurrentThreshold = 120;
    configuration.CurrentLimits.SupplyTimeThreshold = 0.0;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;

    TalonFXFactory.applyAndCheckConfiguration(hardware.motor, configuration, 5);

    /* Absolute */
    CANcoderConfiguration encoderConfiguration = new CANcoderConfiguration();
    encoderConfiguration.MagnetSensor.MagnetOffset = TurretConstants.encoderMagnetOffset;
    encoderConfiguration.MagnetSensor.SensorDirection =
        TurretConstants.encoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;

    hardware.absoluteEncoder.getConfigurator().apply(encoderConfiguration, 0.05);

    /* Zero */
    double absPosition = hardware.absolutePosition.waitForUpdate(0.05).getValue();
    double turretAbsPosition = absPosition / TurretConstants.encoderGearRatio;

    double motorPosition = turretAbsPosition * TurretConstants.gearRatio;

    hardware.motor.setPosition(motorPosition);

    hardware.motor.setNeutralMode(NeutralModeValue.Brake);
  }

  private void configureSwerveHardware() {
    var hardware = HardwareAdapter.SwerveHardware.getInstance();

    // Apply gyro trim to correct Z-axis drift on Pigeon 2 IMU.
    // Configuration details found in Pigeon 2 User's Guide, Section 3.3, Page 15:
    // https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User's%20Guide.pdf

    Pigeon2Configuration gyroConfigs = new Pigeon2Configuration();
    gyroConfigs.GyroTrim.withGyroScalarZ(SwerveConstants.gyroZTrim);
    hardware.gyro.getConfigurator().apply(gyroConfigs);
  }

  private void configureShooterHardware() {
    var hardware = HardwareAdapter.ShooterHardware.getInstance();

    TalonFXConfiguration configuration_right = new TalonFXConfiguration();

    configuration_right.Slot0.kP = ShooterConstants.kP_right;
    configuration_right.Slot0.kI = ShooterConstants.kI_right;
    configuration_right.Slot0.kD = ShooterConstants.kD_right;
    configuration_right.Slot0.kV = ShooterConstants.kF_right;
    configuration_right.Slot0.kS = ShooterConstants.kS_right;

    configuration_right.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    configuration_right.CurrentLimits.SupplyCurrentThreshold = 80;
    configuration_right.CurrentLimits.SupplyTimeThreshold = 0.0;
    //    configuration_right.CurrentLimits.SupplyTimeThreshold = 0.1;
    configuration_right.CurrentLimits.SupplyCurrentLimit = 80;
    configuration_right.CurrentLimits.SupplyCurrentLimitEnable = true;
    TalonFXConfiguration configuration_left = new TalonFXConfiguration();

    configuration_left.Slot0.kP = ShooterConstants.kP_left;
    configuration_left.Slot0.kI = ShooterConstants.kI_left;
    configuration_left.Slot0.kD = ShooterConstants.kD_left;
    configuration_left.Slot0.kV = ShooterConstants.kF_left;
    configuration_left.Slot0.kS = ShooterConstants.kS_left;

    configuration_left.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    configuration_left.CurrentLimits.SupplyCurrentThreshold = 80;
    configuration_left.CurrentLimits.SupplyTimeThreshold = 0.0;
    //    configuration_right.CurrentLimits.SupplyTimeThreshold = 0.1;
    configuration_left.CurrentLimits.SupplyCurrentLimit = 80;
    configuration_left.CurrentLimits.SupplyCurrentLimitEnable = true;

    TalonFXFactory.applyAndCheckConfiguration(hardware.rightMotor, configuration_right, 5);
    TalonFXFactory.applyAndCheckConfiguration(hardware.leftMotor, configuration_left, 5);

    hardware.leftMotor.setInverted(true);

    TalonFXConfiguration configuration_kicker = new TalonFXConfiguration();

    configuration_kicker.Slot0.kP = ShooterConstants.kP_kicker;
    configuration_kicker.Slot0.kI = ShooterConstants.kI_kicker;
    configuration_kicker.Slot0.kD = ShooterConstants.kD_kicker;
    configuration_kicker.Slot0.kV = ShooterConstants.kF_kicker;
    configuration_kicker.Slot0.kS = ShooterConstants.kS_kicker;

    TalonFXFactory.applyAndCheckConfiguration(hardware.kicker, configuration_kicker, 5);
    hardware.kicker.setNeutralMode(NeutralModeValue.Brake);

    hardware.sensorInterrupt.setInterruptEdges(true, true);
  }

  private void configureIntakeHardware() {
    var hardware = HardwareAdapter.IntakeHardware.getInstance();

    TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.CurrentLimits.SupplyCurrentThreshold = IntakeConstants.supplyCurrentThresh;
    configuration.CurrentLimits.SupplyCurrentLimit = IntakeConstants.supplyCurrentLimit;
    configuration.CurrentLimits.SupplyTimeThreshold = IntakeConstants.supplyTriggerTime;
    configuration.CurrentLimits.SupplyCurrentLimitEnable =
        IntakeConstants.supplyCurrentLimitEnabled;

    configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    configuration.OpenLoopRamps.VoltageOpenLoopRampPeriod = IntakeConstants.rampTime;

    TalonFXFactory.applyAndCheckConfiguration(hardware.motor, configuration, 5);
  }

  private void configureArmHardware() {
    var hardware = HardwareAdapter.ArmHardware.getInstance();

    TalonFXConfiguration configuration_big = new TalonFXConfiguration();

    configuration_big.MotionMagic.MotionMagicAcceleration = ArmConstants.cruiseAcceleration_big;
    configuration_big.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.cruiseVelocity_big;

    configuration_big.Slot0.kP = ArmConstants.kP_big;
    configuration_big.Slot0.kI = ArmConstants.kI_big;
    configuration_big.Slot0.kD = ArmConstants.kD_big;
    configuration_big.Slot0.kV = ArmConstants.kF_big;
    configuration_big.Slot0.kS = ArmConstants.kS_big;

    hardware.motorCeo.setPosition(0);

    hardware.motorCeo.setNeutralMode(NeutralModeValue.Brake);
    hardware.motorEmployee1.setNeutralMode(NeutralModeValue.Brake);
    hardware.motorEmployee2.setNeutralMode(NeutralModeValue.Brake);

    TalonFXFactory.applyAndCheckConfiguration(hardware.motorCeo, configuration_big, 5);
  }

  /** Updates the hardware to run with output values of {@link SubsystemBase}'s. */
  void writeHardware(Set<SubsystemBase> enabledSubsystems, @ReadOnly RobotState robotState) {
    if (enabledSubsystems.contains(Swerve.getInstance())) updateSwerve(robotState);
    if (enabledSubsystems.contains(Shooter.getInstance())) updateShooter(robotState);
    if (enabledSubsystems.contains(Turret.getInstance())) updateTurret(robotState);
    if (enabledSubsystems.contains(Intake.getInstance())) updateIntake(robotState);
    if (enabledSubsystems.contains(Arm.getInstance())) updateArm(robotState);
    if (enabledSubsystems.contains(Lighting.getInstance())) updateLighting(robotState);
    updateJoystick(robotState);

    configurator.setIgnoredTags(robotState.ignoredTags);
  }

  private void updateJoystick(RobotState state) {
    var hardware = HardwareAdapter.JoystickHardware.getInstance();
    hardware.driverXboxController.setRumble(GenericHID.RumbleType.kBothRumble, state.rumbleDriver);
    hardware.operatorXboxController.setRumble(
        GenericHID.RumbleType.kBothRumble, state.rumbleOperator);
  }

  private void updateSwerve(RobotState state) {
    var hardware = HardwareAdapter.SwerveHardware.getInstance();
    var outputs = Swerve.getInstance().getOutputs();

    if (!outputs.isIdle()) {

      hardware.FL.setDesiredState(outputs.getStates()[0], hardware.modules[0].getState(), false);
      hardware.FR.setDesiredState(outputs.getStates()[1], hardware.modules[1].getState(), false);
      hardware.BL.setDesiredState(outputs.getStates()[2], hardware.modules[2].getState(), false);
      hardware.BR.setDesiredState(outputs.getStates()[3], hardware.modules[3].getState(), false);
    }
  }

  private void updateArm(RobotState state) {
    var hardware = HardwareAdapter.ArmHardware.getInstance();
    var outputs_big = Arm.getInstance().getOutputsBig();

    SmartDashboard.putNumber("Arm wanted position", outputs_big.getReference());

    Logger.getInstance().log("Arm/Arm wanted position", outputs_big.getReference());

    hardware.motorCeo.setOutput(outputs_big, false);
    state.armPoseRef = outputs_big.getReference();
  }

  private void updateLighting(RobotState state) {
    var hardware = HardwareAdapter.LightingHardware.getInstance();
    if (Lighting.getInstance().getState() != state.lastSet) {
      hardware.candle.animate(Lighting.getInstance().getOutput());
      state.lastSet = Lighting.getInstance().getState();
    }
  }

  private void updateIntake(RobotState state) {
    var hardware = HardwareAdapter.IntakeHardware.getInstance();
    var outputs = Intake.getInstance().getOutputs();

    hardware.motor.setOutput(outputs, false);
  }

  private void updateShooter(RobotState state) {
    var hardware = HardwareAdapter.ShooterHardware.getInstance();

    var outputs1 = Shooter.getInstance().getOutputs1();
    var outputs2 = Shooter.getInstance().getOutputs2();

    var kickerOut = Shooter.getInstance().getOutputsKicker();

    var blowerOut = Shooter.getInstance().getBlowerOutPercentage();

    hardware.rightMotor.setOutput(outputs1, false);
    hardware.leftMotor.setOutput(outputs2, false);

    state.rightVRef = outputs1.getReference();
    state.leftVRef = outputs2.getReference();

    SmartDashboard.putNumber("SET/kicker out", kickerOut.getReference());
    SmartDashboard.putString("SET/kicker out mode", kickerOut.getControlMode().name());

    hardware.kicker.setOutput(kickerOut, false);

    hardware.blower.set(TalonSRXControlMode.PercentOutput, blowerOut);
  }

  private void updateTurret(RobotState state) {
    var hardware = HardwareAdapter.TurretHardware.getInstance();
    var outputs = Turret.getInstance().getMotorOutputs();

    SmartDashboard.putNumber("Turret/Wanted turret", outputs.getReference());
    Logger.getInstance().log("Turret/Wanted turret", outputs.getReference());

    hardware.motor.setOutput(outputs, false);
    state.turretMotorPositionRef = outputs.getReference();
  }
}
