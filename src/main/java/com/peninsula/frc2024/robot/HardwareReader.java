package com.peninsula.frc2024.robot;

import com.peninsula.frc2024.config.FieldConstants;
import com.peninsula.frc2024.logging.Logger;
import com.peninsula.frc2024.robot.HardwareAdapter.JoystickHardware;
import com.peninsula.frc2024.subsystems.*;
import com.peninsula.frc2024.util.Util;
import com.peninsula.frc2024.util.peninsulaCoolios.phronesis.fieldAwareness.FieldNoteSim;
import com.peninsula.frc2024.vision.LimelightHelpers;
import com.peninsula.frc2024.vision.ProjectionHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Set;
import org.ejml.simple.SimpleMatrix;

public class HardwareReader {

  public HardwareReader() {}

  /**
   * Takes all of the sensor data from the hardware, and unwraps it into the current {@link
   * RobotState}.
   */
  void readState(Set<SubsystemBase> enabledSubsystems, RobotState state) {
    Robot.sLoopDebugger.reset();
    readGameAndFieldState(state);
    Robot.sLoopDebugger.addPoint("Read game");
    if (state.gamePeriod == RobotState.GamePeriod.TELEOP) readJoystickState(state);
    Robot.sLoopDebugger.addPoint("Read joystick");
    if (enabledSubsystems.contains(Arm.getInstance())) readArmState(state);
    Robot.sLoopDebugger.addPoint("Read Arm");
    if (enabledSubsystems.contains(Turret.getInstance())) readTurretState(state);
    Robot.sLoopDebugger.addPoint("Read Turret");
    if (enabledSubsystems.contains(Intake.getInstance())) readIntakeState(state);
    Robot.sLoopDebugger.addPoint("Read Intake");
    Robot.sLoopDebugger.finish();
    if (enabledSubsystems.contains(Shooter.getInstance())) readShooterState(state);

    readLimelightState(state);
  }

  TimeInterpolatableBuffer<Pose2d> poses =
      TimeInterpolatableBuffer.createBuffer(this::interpolate, 2.0);

  public Pose2d interpolate(Pose2d start, Pose2d endValue, double t) {
    if (t < 0) {
      return start;
    } else if (t >= 1) {
      return endValue;
    } else {
      var twist = start.log(endValue);
      var scaledTwist = new Twist2d(twist.dx * t, twist.dy * t, twist.dtheta * t);
      return start.exp(scaledTwist);
    }
  }

  FieldNoteSim sim = new FieldNoteSim();

  private void readLimelightState(RobotState state) {
    LimelightHelpers.setPipelineIndex("", 0);
    state.limelightResults = LimelightHelpers.getLatestResults("");

    poses.addSample(Timer.getFPGATimestamp(), state.lastEst);

    if (Robot.isSimulation()) {
      for (int i = 1; i <= 5; i++) {
        if (!sim.isGone(i, 15 - Timer.getMatchTime())) {
          //          if (i != 4) {
          state.fieldObserver.addPieceSeenObservationAuto(
              FieldConstants.GamePieceLocations.centerline[i - 1],
              state.state,
              Timer.getFPGATimestamp());
          //          } else {
          //            state.fieldObserver.addPieceSeenObservationAuto(
          //                FieldConstants.GamePieceLocations.centerline[i - 1].plus(
          //                    new Transform2d(0, 0.4, new Rotation2d())),
          //                state.state,
          //                Timer.getFPGATimestamp());
          //          }
        }
      }
    }

    if (state.limelightResults.targetingResults.targets_Detector.length > 0) {
      for (int i = 0; i < state.limelightResults.targetingResults.targets_Detector.length; i++) {
        LimelightHelpers.LimelightTarget_Detector detection =
            state.limelightResults.targetingResults.targets_Detector[i];

        double laten =
            (state.limelightResults.targetingResults.latency_pipeline
                    + state.limelightResults.targetingResults.latency_pipeline
                    + state.limelightResults.targetingResults.latency_jsonParse)
                / 1000.0;

        double pixel_x = detection.tx_pixels, pixel_y = detection.ty_pixels;

        SmartDashboard.putNumber("pixel_x", pixel_x);
        SmartDashboard.putNumber("pixel_y", pixel_y);

        SimpleMatrix noteLocation = ProjectionHelpers.project(pixel_x, pixel_y);

        Transform2d noteOffset =
            new Transform2d(-noteLocation.get(1), noteLocation.get(0), new Rotation2d(Math.PI));

        double time = Timer.getFPGATimestamp() - laten;

        boolean noteT = poses.getSample(time).isPresent();

        if (noteT) {
          Pose2d note = poses.getSample(time).get().plus(noteOffset);

          state.m_field.getObject("note " + (i + 1)).setPose(note);

          if (state.gamePeriod == RobotState.GamePeriod.AUTO) {
            state.fieldObserver.addPieceSeenObservationAuto(
                note, state.state, Timer.getFPGATimestamp());
          } else {
            state.fieldObserver.addUnstagedNote(note, state.state, Timer.getFPGATimestamp());
          }
        }
      }
    }
  }

  private void readIntakeState(RobotState state) {
    var hardware = HardwareAdapter.IntakeHardware.getInstance();

    //    double currentSupply = hardware.motor.getSupplyCurrent().getValue();
    //
    //    SmartDashboard.putNumber(
    //        "Currents/Intake current stator", hardware.motor.getStatorCurrent().getValue());
    //    SmartDashboard.putNumber("Currents/Intake current supply", currentSupply);
    //
    //    Logger.getInstance().log("Intake/Currents/Intake current supply", currentSupply);

    double velo = hardware.motor.getVelocity().getValue();

    SmartDashboard.putNumber("Intake/intake velo", velo);

    Logger.getInstance().log("Intake/intake velo", velo);
  }

  private void readShooterState(RobotState state) {
    var hardware = HardwareAdapter.ShooterHardware.getInstance();

    state.rightV = hardware.rightMotor.getVelocity().getValue();
    state.leftV = hardware.leftMotor.getVelocity().getValue();

    SmartDashboard.putNumber("Shooter/Right Speed", state.rightV);
    SmartDashboard.putNumber("Shooter/Right Wanted", state.rightVRef);
    SmartDashboard.putNumber("Shooter/Left Speed", state.leftV);
    SmartDashboard.putNumber("Shooter/Left Wanted", state.leftVRef);
    SmartDashboard.putNumber("Shooter/Kicker Speed", hardware.kicker.getVelocity().getValue());

    SmartDashboard.putNumber(
        "Shooter/Left Current", hardware.leftMotor.getSupplyCurrent().getValue());
    SmartDashboard.putNumber(
        "Shooter/Right Current", hardware.rightMotor.getSupplyCurrent().getValue());

    Logger.getInstance().log("Shooter/Kicker Speed", hardware.kicker.getVelocity().getValue());
    Logger.getInstance().log("Shooter/Right Speed", state.rightV);
    Logger.getInstance().log("Shooter/Right Wanted", state.rightVRef);
    Logger.getInstance().log("Shooter/Left Speed", state.leftV);
    Logger.getInstance().log("Shooter/Left Wanted", state.leftVRef);

    //    double supply =
    //        hardware.motor1.getSupplyCurrent().getValue()
    //            + hardware.motor2.getSupplyCurrent().getValue();
    //
    //    SmartDashboard.putNumber("Currents/Shooter supply", supply);
    //
    //    Logger.getInstance().log("Shooter/Currents/total supply", supply);
    //
    //    SmartDashboard.putNumber(
    //        "Voltages/Shooter right", hardware.motor1.getMotorVoltage().getValue());
    //    SmartDashboard.putNumber(
    //        "Voltages/Shooter left", hardware.motor2.getMotorVoltage().getValue());

    boolean hadPiece = state.pieceInKick;
    state.pieceInKick = !hardware.sensor.get();

    if (hadPiece && !state.pieceInKick) {
      state.pieceLeftTime = Timer.getFPGATimestamp();
    }

    boolean backSensor = !hardware.backKickerSensor.get();
    if (!state.backSensor && backSensor) {
      state.backSensorOnTime = Timer.getFPGATimestamp();
    }
    state.backSensor = backSensor;

    state.timeOfLastFallingEdge = hardware.sensorInterrupt.getFallingTimestamp();
    state.timeOfLastRisingEdge = hardware.sensorInterrupt.getRisingTimestamp();

    SmartDashboard.putBoolean("Shooter/Front Sensor", state.pieceInKick);
    SmartDashboard.putBoolean("Shooter/Back Sensor", state.backSensor);

    Logger.getInstance().log("Shooter/Front Sensor", state.pieceInKick);
    Logger.getInstance().log("Shooter/Back Sensor", state.backSensor);
  }

  private void readTurretState(RobotState state) {
    var hardware = HardwareAdapter.TurretHardware.getInstance();
    state.turretMotorPosition = hardware.motor.getPositionValue();

    SmartDashboard.putNumber("Turret/real position", state.turretMotorPosition);
    Logger.getInstance().log("Turret/real position", state.turretMotorPosition);

    //    double currentSupply = hardware.motor.getSupplyCurrent().getValue();
    //
    //    SmartDashboard.putNumber("Currents/turret supply", currentSupply);
    //    Logger.getInstance().log("Turret/Currents/turret supply", currentSupply);

    state.turretAbsoluteEncoderPosition = hardware.absolutePosition.refresh().getValue();

    SmartDashboard.putNumber("Turret/Encoder Position", state.turretAbsoluteEncoderPosition);
  }

  private void readArmState(RobotState state) {
    var hardware = HardwareAdapter.ArmHardware.getInstance();

    state.armPos = hardware.motorCeo.getPositionValue();

    if (!Robot.isRobotReal()) {
      var outputs_big = Arm.getInstance().getOutputsBig();

      state.armPos = outputs_big.getReference();
    }

    state.shooterTrim = Robot.onBlueAlliance ? state.shooterTrimBlue : state.shooterTrimRed;

    SmartDashboard.putNumber("Arm/shooter trim", state.shooterTrim);
    Logger.getInstance().log("Arm/shooter trim", state.shooterTrim);

    SmartDashboard.putNumber("Arm/Position", state.armPos);
    SmartDashboard.putNumber("Arm/Position Red", state.armPoseRef);
    Logger.getInstance().log("Arm/Position", state.armPos);

    //    double currentSupply1 = hardware.motorCeo.getSupplyCurrent().getValue();
    //    double currentSupply2 = hardware.motorEmployee1.getSupplyCurrent().getValue();
    //    double currentSupply3 = hardware.motorEmployee2.getSupplyCurrent().getValue();
    //
    //    SmartDashboard.putNumber("Currents/turret supply1", currentSupply1);
    //    SmartDashboard.putNumber("Currents/turret supply2", currentSupply2);
    //    SmartDashboard.putNumber("Currents/turret supply3", currentSupply3);
    //
    //    Logger.getInstance().log("Turret/Currents/turret supply1", currentSupply1);
    //    Logger.getInstance().log("Turret/Currents/turret supply2", currentSupply2);
    //    Logger.getInstance().log("Turret/Currents/turret supply3", currentSupply3);

    RobotStateEstimator.getInstance().updateSuperstructureViz(state);
  }

  private void readGameAndFieldState(RobotState state) {
    state.gameData = DriverStation.getGameSpecificMessage();
    state.gameTimeS = Timer.getFPGATimestamp();

    Logger.getInstance().log("Field/Robot", state.lastEst);
  }

  private void readJoystickState(RobotState state) {
    var hardware = JoystickHardware.getInstance();

    state.driverLeftX = Util.handleDeadBand(hardware.driverXboxController.getLeftX(), 0.04);
    state.driverLeftY = Util.handleDeadBand(hardware.driverXboxController.getLeftY(), 0.04);
    state.driverRightY = Util.handleDeadBand(hardware.driverXboxController.getRightY(), 0.09);
    state.driverRightX = Util.handleDeadBand(hardware.driverXboxController.getRightX(), 0.04);
    state.driverRt = hardware.driverXboxController.getRightTriggerAxis();
    state.operatorAPressed = hardware.operatorXboxController.getAButton();
    state.operatorRtPressed = hardware.operatorXboxController.getRightTriggerAxis() > 0.1;
    state.operatorLtPressed = hardware.operatorXboxController.getLeftTriggerAxis() > 0.5;
    state.operatorRbPressed = hardware.operatorXboxController.getRightBumper();
    state.operatorLbPressed = hardware.operatorXboxController.getLeftBumper();
    state.operatorDPadLeftPressed = hardware.operatorXboxController.getLeftStickButton();
    state.operatorDPadRightPressed = hardware.operatorXboxController.getRightStickButton();
    state.driverAPressed = hardware.driverXboxController.getAButton();
    state.driverBPressed = hardware.driverXboxController.getBButtonPressed();
    state.driverXPressed = hardware.driverXboxController.getXButton();
    state.driverYPressed = hardware.driverXboxController.getYButton();
    state.operatorLeftX = Util.handleDeadBand(hardware.operatorXboxController.getLeftX(), 0.09);
    state.operatorLeftY = Util.handleDeadBand(hardware.operatorXboxController.getLeftY(), 0.09);
    state.operatorRightX = Util.handleDeadBand(hardware.operatorXboxController.getRightX(), 0.2);
    state.operatorRightY = Util.handleDeadBand(hardware.operatorXboxController.getRightY(), 0.2);
    state.operatorAPressed = hardware.operatorXboxController.getAButton();
    state.operatorB = hardware.operatorXboxController.getBButton();
    state.operatorBPressed = hardware.operatorXboxController.getBButtonPressed();
    state.operatorXPressed = hardware.operatorXboxController.getXButton();
    state.operatorYPressed = hardware.operatorXboxController.getYButton();
    state.driverLtPressed = hardware.driverXboxController.getLeftTriggerAxis() > 0.5;
    state.driverLbPressed = hardware.driverXboxController.getLeftBumper();
    state.driverRbPressed = hardware.driverXboxController.getRightBumper();

    state.driverDPadUp = hardware.driverXboxController.getPOV() == 0;
    state.driverDPadDown = hardware.driverXboxController.getPOV() == 180;
    state.driverDPadRight = hardware.driverXboxController.getPOV() == 90;
    state.driverDPadLeft = hardware.driverXboxController.getPOV() == 270;

    state.operatorDPadUp = hardware.operatorXboxController.getPOV() == 0;
    state.operatorDPadDown = hardware.operatorXboxController.getPOV() == 180;
    state.operatorDPadRight = hardware.operatorXboxController.getPOV() == 90;
    state.operatorDPadLeft = hardware.operatorXboxController.getPOV() == 270;
  }
}
