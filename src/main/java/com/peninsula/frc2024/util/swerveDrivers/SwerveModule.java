package com.peninsula.frc2024.util.swerveDrivers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.peninsula.frc2024.config.SwerveConstants;
import com.peninsula.frc2024.util.CTREConfigs;
import com.peninsula.frc2024.util.SwerveModuleConstants;
import com.peninsula.frc2024.util.config.TalonFXFactory;
import com.peninsula.frc2024.util.control.CANcoderPro;
import com.peninsula.frc2024.util.control.TalonFXController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  public int moduleNumber;
  public double angleOffset;
  private TalonFXController mAngleMotor;
  private TalonFXController mDriveMotor;
  private CANcoderPro angleEncoder;
  private double lastAngle;

  SwerveModuleConstants constants;

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          SwerveConstants.Constants.Swerve.driveKS,
          SwerveConstants.Constants.Swerve.driveKV,
          SwerveConstants.Constants.Swerve.driveKA);

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> steerPosition;
  private final StatusSignal<Double> steerVelocity;

  public final VelocityTorqueCurrentFOC torqueDriveSetter;
  public final VelocityVoltage voltageDriveSetterFOC;
  public final VelocityVoltage voltageDriveSetter;
  public final PositionTorqueCurrentFOC torqueAngleSetter;

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;
    this.constants = moduleConstants;

    /* Angle Motor Config */
    mAngleMotor =
        TalonFXFactory.createDefaultFalconPro(moduleConstants.angleMotorID, "angle motor");
    configAngleMotor();

    /* Drive Motor Config */
    mDriveMotor =
        TalonFXFactory.createDefaultFalconPro(moduleConstants.driveMotorID, "drive motor");
    configDriveMotor();

    torqueDriveSetter = new VelocityTorqueCurrentFOC(0);
    torqueDriveSetter.UpdateFreqHz = 0;

    voltageDriveSetterFOC = new VelocityVoltage(0).withEnableFOC(true);
    voltageDriveSetterFOC.UpdateFreqHz = 0;

    voltageDriveSetter = new VelocityVoltage(0).withEnableFOC(false);
    voltageDriveSetter.UpdateFreqHz = 0;

    torqueAngleSetter = new PositionTorqueCurrentFOC(0);
    torqueAngleSetter.UpdateFreqHz = 0;

    drivePosition = mDriveMotor.getPosition();
    driveVelocity = mDriveMotor.getVelocity();
    steerPosition = mAngleMotor.getPosition();
    steerVelocity = mAngleMotor.getVelocity();

    /* Angle Encoder Config */
    angleEncoder = new CANcoderPro(moduleConstants.cancoderID, "swerve");
    configAngleEncoder();

    lastAngle = getState().angle.getDegrees();
  }

  public void setDesiredState(
      SwerveModuleState desiredState, SwerveModuleState currentModuleState, boolean isOpenLoop) {
    //    desiredState =
    //        CTREModuleState.optimize(
    //            desiredState,
    //            currentModuleState
    //                .angle); // Custom optimize command, since default WPILib optimize assumes

    desiredState = SwerveModuleState.optimize(desiredState, currentModuleState.angle);

    if (isOpenLoop) {
      double percentOutput =
          desiredState.speedMetersPerSecond / SwerveConstants.Constants.Swerve.maxSpeed;
      mDriveMotor.set(percentOutput);
    } else {
      double velocity =
          SwerveConstants.Constants.Swerve.driveGearRatio
              * (desiredState.speedMetersPerSecond
                  / SwerveConstants.Constants.Swerve.wheelCircumference);
      SmartDashboard.putNumber("Swerve/wanted velocity rps " + moduleNumber, velocity);

      if (Math.abs(velocity) > SwerveConstants.Constants.Swerve.maxSpeed * 0.7) {
        mDriveMotor.setControl(voltageDriveSetter.withVelocity(velocity));
      } else {
        mDriveMotor.setControl(voltageDriveSetterFOC.withVelocity(velocity));
      }
      //      mDriveMotor.setControl(torqueDriveSetter.withVelocity(velocity));
      //      mDriveMotor.set(-1);
    }

    double angle =
        (Math.abs(desiredState.speedMetersPerSecond)
                <= (SwerveConstants.Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle
                .getRotations(); // Prevent rotating module if speed is less than 1%. Prevents
    // jittering.

    mAngleMotor.setControl(torqueAngleSetter.withPosition(angle));
    lastAngle = angle;
    a++;
  }

  int a = 0;

  public boolean cancoderSet = false;
  public boolean driveSet = false;
  public boolean angleSet = false;

  private void configAngleEncoder() {
    if (cancoderSet) return;

    cancoderSet =
        TalonFXFactory.checkErrorAndRetry(
            () ->
                angleEncoder.getConfigurator().apply(CTREConfigs.swerveCancoderConfig(angleOffset)),
            5);
  }

  private void configAngleMotor() {
    if (angleSet) return;

    TalonFXConfiguration angleConfig = new TalonFXConfiguration();

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = SwerveConstants.Constants.Swerve.angleKP;
    slot0.kI = SwerveConstants.Constants.Swerve.angleKI;
    slot0.kD = SwerveConstants.Constants.Swerve.angleKD;
    slot0.kV = SwerveConstants.Constants.Swerve.angleKV;
    slot0.kS = SwerveConstants.Constants.Swerve.angleKS;
    angleConfig.Slot0 = slot0;

    angleConfig.MotorOutput.NeutralMode = SwerveConstants.Constants.Swerve.angleNeutralModePro;
    angleConfig.MotorOutput.Inverted =
        SwerveConstants.Constants.Swerve.angleMotorInvert
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    angleConfig.Feedback.FeedbackRemoteSensorID = constants.cancoderID;

    angleConfig.Feedback.RotorToSensorRatio = SwerveConstants.Constants.Swerve.angleGearRatio;

    angleConfig.ClosedLoopGeneral.ContinuousWrap = true;

    angleConfig.CurrentLimits.SupplyTimeThreshold = 0.0;
    angleConfig.CurrentLimits.SupplyCurrentThreshold = 120.0;
    angleConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    angleConfig.CurrentLimits.SupplyCurrentLimit = 120.0;

    angleConfig.CurrentLimits.StatorCurrentLimit = 120.0;
    angleConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    angleSet = TalonFXFactory.applyAndCheckConfiguration(mAngleMotor, angleConfig, 5);

    SmartDashboard.putBoolean("Swerve/AngleMod" + moduleNumber, angleSet);
  }

  private void configDriveMotor() {
    if (driveSet) return;

    TalonFXConfiguration a = CTREConfigs.swerveDriveFXConfig();

    a.MotorOutput.NeutralMode = SwerveConstants.Constants.Swerve.driveNeutralModePro;
    a.MotorOutput.Inverted =
        SwerveConstants.Constants.Swerve.driveMotorInvert
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    a.CurrentLimits.SupplyTimeThreshold = 0.0;
    a.CurrentLimits.SupplyCurrentThreshold = 120.0;
    a.CurrentLimits.SupplyCurrentLimitEnable = true;
    a.CurrentLimits.SupplyCurrentLimit = 100.0;

    a.CurrentLimits.StatorCurrentLimit = 100.0;
    a.CurrentLimits.StatorCurrentLimitEnable = true;

    a.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.02;
    a.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
    a.OpenLoopRamps.TorqueOpenLoopRampPeriod = 0.02;

    a.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    a.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    a.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.02;

    driveSet = TalonFXFactory.applyAndCheckConfiguration(mDriveMotor, a, 5);

    driveSet =
        driveSet && TalonFXFactory.checkErrorAndRetry(() -> mDriveMotor.setPosition(0, 0.05), 5);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePositionValue() * 360.0);
  }

  // Edit from CTRE-swerve
  public SwerveModulePosition getPosition(boolean refresh) {
    if (refresh) {
      drivePosition.refresh();
      driveVelocity.refresh();
      steerPosition.refresh();
      steerVelocity.refresh();
    }

    double drive_rot = BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity);
    double angle_rot = BaseStatusSignal.getLatencyCompensatedValue(steerPosition, steerVelocity);

    SmartDashboard.putNumber(
        "Swerve/measured drive velo rps " + moduleNumber, driveVelocity.getValue());

    //    SmartDashboard.putNumber("current1 draw " + moduleNumber,
    // mDriveMotor.getStatorCurrent().getValue());
    //    SmartDashboard.putNumber("current2 draw " + moduleNumber,
    // mDriveMotor.getSupplyCurrent().getValue());
    //    SmartDashboard.putNumber("current3 draw " + moduleNumber,
    // mDriveMotor.getTorqueCurrent().getValue());

    //    SmartDashboard.putNumber("voltage applied m " + moduleNumber,
    // mDriveMotor.getMotorVoltage().refresh().getValue());
    //    SmartDashboard.putNumber("voltage applied s " + moduleNumber,
    // mDriveMotor.getSupplyVoltage().refresh().getValue());
    //    SmartDashboard.putNumber("closed loop err " + moduleNumber,
    // mDriveMotor.getClosedLoopError().getValue());

    /*
     * TODO: Verify this works (from CTRE)
     * Back out the drive rotations based on angle rotations due to coupling between
     * azimuth and steer
     */
    //    drive_rot -= angle_rot * m_couplingRatioDriveRotorToCANcoder;

    SwerveModulePosition pos = new SwerveModulePosition();

    pos.distanceMeters =
        drive_rot
            / SwerveConstants.Constants.Swerve.driveGearRatio
            * SwerveConstants.Constants.Swerve.wheelCircumference;
    pos.angle = Rotation2d.fromRotations(angle_rot);

    return pos;
  }

  // TODO: probably incorrect but nothing is really based on this method yet
  public SwerveModuleState getState() {
    double velocity =
        Conversions.falconToMPS(
            mDriveMotor.getRotorVelocityValue(),
            SwerveConstants.Constants.Swerve.wheelCircumference,
            SwerveConstants.Constants.Swerve.driveGearRatio);
    Rotation2d angle =
        Rotation2d.fromRotations(
            BaseStatusSignal.getLatencyCompensatedValue(steerPosition, steerVelocity));

    //    double currentSupply = mDriveMotor.getSupplyCurrent().refresh().getValue();
    //
    //    SmartDashboard.putNumber("Currents/module supply" + moduleNumber, currentSupply);
    //    Logger.getInstance().log("Swerve/Currents/module supply" + moduleNumber, currentSupply);

    return new SwerveModuleState(velocity, angle);
  }

  /**
   * @return drive position, drive velocity, angle position, angle velocity
   */
  public BaseStatusSignal[] getBaseSignals() {
    return new StatusSignal[] {drivePosition, driveVelocity, steerPosition, steerVelocity};
  }
}
