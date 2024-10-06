package com.peninsula.frc2024.robot;

import com.peninsula.frc2024.config.*;
import com.peninsula.frc2024.logging.Logger;
import com.peninsula.frc2024.subsystems.*;
import com.peninsula.frc2024.subsystems.controllers.drive.FSDController;
import com.peninsula.frc2024.util.Flipper;
import com.peninsula.frc2024.util.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.IOException;
import java.util.Set;

/** Used to produce {@link Commands}'s from human input. Should only be used in robot package. */
public class Control {

  /** Modifies commands based on operator input devices. */
  void updateCommands(Commands commands, @ReadOnly RobotState state) throws IOException {
    updateDriveCommands(commands, state);
    updateSuperstructureCommands(commands, state);
  }

  private void updateDriveCommands(Commands commands, RobotState state) {

    if (!state.runningFSD) {
      commands.swerveWanted = Swerve.State.TELEOP;
      FSDController.ampMode = false;
      state.ignoredTags = Set.of(13, 14);
    }

    if (state.runningFSD && FSDController.targetReached()) {
      state.runningFSD = false;
    }

    if (state.runningFSD && state.climbMode && state.driverAPressed) {
      state.runningFSD = false;
      state.climbUp = false;
    }

    if (state.driverDPadDown) {
      state.runningFSD = false;
    }

    if (Math.abs(state.driverRightX) > 0.1
        || Math.hypot(state.driverLeftX, state.driverLeftY) > 0.1) {
      if (!state.driverFSDLock) {
        state.runningFSD = false;
      }
    } else {
      state.driverFSDLock = false;
    }

    if (state.runningFSD)
      SmartDashboard.putBoolean("FSD Target reached", FSDController.targetReached());
    else SmartDashboard.putBoolean("FSD Target reached", true);

    if (!state.runningFSD) {
      // Initializes FSD for amp
      if (state.driverRbPressed) {
        double angle = state.lastEst.getRotation().getRotations();
        double roundedAngle = Math.round(angle * 4) / 4.0;

        if (roundedAngle == -0.25) {
          // Check if the original angle is closer to 0.25 or -0.25 and adjust accordingly
          double distanceToQuarter = Math.abs(angle);
          double distanceToMinusQuarter = Math.abs(angle + 0.5);
          if (distanceToQuarter < distanceToMinusQuarter) {
            roundedAngle = 0.0;
          } else {
            roundedAngle = 0.5;
          }
        }
        if (Robot.onBlueAlliance)
          roundedAngle = Flipper.flipRot(Rotation2d.fromRotations(roundedAngle)).getRotations();
        tryStartFSDTarget(
            commands,
            state,
            new Pose2d(
                FieldConstants.ampFSDRedFarTarget.getTranslation(),
                Rotation2d.fromRotations(roundedAngle)),
            new Rotation2d());
        FSDController.ampMode = true;
      }
      // Climb Far side
      if (state.driverYPressed) {
        tryStartFSDTarget(commands, state, FieldConstants.climbFarSideFarRed, new Rotation2d());
        setSecondaryFSDTarget(commands, state, FieldConstants.climbFarSideFSDTargetRed);
        state.ignoredTags =
            Robot.onBlueAlliance
                ? Set.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13)
                : Set.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 14, 15, 16);
        state.climbMode = true;
      }
      // Climb Amp side
      else if (state.driverXPressed && Robot.onBlueAlliance
          || state.driverBPressed && !Robot.onBlueAlliance) {
        tryStartFSDTarget(
            commands,
            state,
            FieldConstants.climbAmpSideFarRed,
            Rotation2d.fromDegrees(Robot.onBlueAlliance ? 60 : -60));
        setSecondaryFSDTarget(commands, state, FieldConstants.climbAmpSideFSDTargetRed);
        state.ignoredTags =
            Robot.onBlueAlliance
                ? Set.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13)
                : Set.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 14, 15, 16);
        state.climbMode = true;
      }
      // Climb Source side
      else if (state.driverXPressed && !Robot.onBlueAlliance
          || state.driverBPressed && Robot.onBlueAlliance) {
        tryStartFSDTarget(
            commands,
            state,
            FieldConstants.climbSourceSideFarRed,
            Rotation2d.fromDegrees(Robot.onBlueAlliance ? -60 : 60));
        setSecondaryFSDTarget(commands, state, FieldConstants.climbSourceSideFSDTargetRed);
        state.ignoredTags =
            Robot.onBlueAlliance
                ? Set.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13)
                : Set.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 14, 15, 16);
        state.climbMode = true;
      }
      // Trap Far side
      if (state.driverDPadUp) {
        tryStartFSDTarget(
            commands, state, FieldConstants.trapFarSideFSDTargetRed, new Rotation2d());
        state.ignoredTags =
            Robot.onBlueAlliance
                ? Set.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13)
                : Set.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 14, 15, 16);
      }
      // Trap Amp side
      else if (state.driverDPadLeft && Robot.onBlueAlliance
          || state.driverDPadRight && !Robot.onBlueAlliance) {
        tryStartFSDTarget(
            commands,
            state,
            FieldConstants.trapAmpSideFSDTargetRed,
            Rotation2d.fromDegrees(Robot.onBlueAlliance ? 60 : -60));
        state.ignoredTags =
            Robot.onBlueAlliance
                ? Set.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13)
                : Set.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 14, 15, 16);
      }
      // Trap Source side
      else if (state.driverDPadLeft && !Robot.onBlueAlliance
          || state.driverDPadRight && Robot.onBlueAlliance) {
        tryStartFSDTarget(
            commands,
            state,
            FieldConstants.trapSourceSideFSDTargetRed,
            Rotation2d.fromDegrees(Robot.onBlueAlliance ? -60 : 60));
        state.ignoredTags =
            Robot.onBlueAlliance
                ? Set.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13)
                : Set.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 14, 15, 16);
      }
    }

    commands.boostWanted = (state.driverRt > 0.2);
    commands.robotCentricWanted = state.driverLtPressed;

    SmartDashboard.putBoolean("Swerve/FSDEnabled", commands.swerveWanted == Swerve.State.FSD);

    Pose2d ghost = commands.wantedFSDPose;

    state.m_field.getObject("ghostPose").setPose(ghost);
    Logger.getInstance().log("Field/ghostPose", ghost);
  }

  /**
   * Initiates full self-driving (FSD) for the robot to navigate to a specified location.
   *
   * @param commands
   * @param state
   * @param wantedPose The desired pose (position and orientation) where the robot should navigate
   *     to on the Red Alliance.
   */
  private void tryStartFSDTarget(
      Commands commands, RobotState state, Pose2d wantedPose, Rotation2d wantedAngle) {
    if (Robot.onBlueAlliance) {
      wantedPose = Flipper.flip(wantedPose);
    }
    commands.wantedFSDPose = wantedPose;
    commands.wantedFSDLaneAngle = wantedAngle;
    state.runningFSD = true;
    FSDController.resetController = true;
    commands.swerveWanted = Swerve.State.FSD;
    state.driverFSDLock = true;
  }

  private void setSecondaryFSDTarget(Commands commands, RobotState state, Pose2d secondaryPose) {
    if (Robot.onBlueAlliance) {
      secondaryPose = Flipper.flip(secondaryPose);
    }
    commands.secondaryFSDGoal = secondaryPose;
  }

  private void updateSuperstructureCommands(Commands commands, RobotState state) {
    commands.turretWanted = Turret.State.AIM_SHOOT_ON_MOVE;
    commands.wantedArm = Arm.State.SHOOTING_AIM;
    commands.shooterWanted = Shooter.State.IDLE;
    commands.kickerWanted = Shooter.KickerState.HOLD;
    commands.intakeWanted = Intake.State.OFF;
    commands.blowerWanted = Shooter.BlowerState.OFF;

    if (state.operatorDPadUp && !state.lastDPadPressed) {
      if (Robot.onBlueAlliance) state.shooterTrimBlue += 0.0005;
      else {
        state.shooterTrimRed += 0.0005;
      }
    } else if (state.operatorDPadDown && !state.lastDPadPressed) {
      if (Robot.onBlueAlliance) state.shooterTrimBlue -= 0.0005;
      else {
        state.shooterTrimRed -= 0.0005;
      }
    }
    state.lastDPadPressed = state.operatorDPadUp || state.operatorDPadDown;

    SmartDashboard.putNumber("Shooter/pieceLeftTime", state.pieceLeftTime);
    SmartDashboard.putNumber("gameTimeS", state.gameTimeS);

    if (state.pieceInKick) {
      commands.wantedLighting = Lighting.State.HAS_NOTE;
    } else {
      commands.wantedLighting = Lighting.State.IDLE;

      if (state.gameTimeS - state.pieceLeftTime > ShooterConstants.keepShootingTime) {
        commands.turretWanted = Turret.State.INTAKE;
        commands.wantedArm = Arm.State.INTAKE;
      }
    }

    // Aiming Turret/shooter for trap
    if (state.operatorDPadRight) {
      commands.turretWanted = Turret.State.TRAP;
      commands.wantedArm = Arm.State.TRAP;
      commands.shooterWanted = Shooter.State.TRAP;
      commands.blowerWanted = Shooter.BlowerState.ON;
    }

    /* Intake controls */
    if (state.operatorAPressed || (state.backSensor && !state.pieceInKick)) {
      state.climbMode = false;
      state.climbUp = false;
      if (!state.pieceInKick) {
        if (Math.abs(state.turretMotorPosition) < ShootingConstants.maxIntakeRunTurretError) {
          commands.intakeWanted = Intake.State.RUN;
        }
        commands.turretWanted = Turret.State.INTAKE;
        commands.wantedArm = Arm.State.INTAKE;
        commands.wantedLighting = Lighting.State.INTAKING;
      }
    }
    if (state.pieceInKick || state.backSensor) {
      commands.wantedLighting = Lighting.State.HAS_NOTE;
    }

    if (state.backSensor && Timer.getFPGATimestamp() - state.backSensorOnTime < 1.0) {
      state.rumbleDriver = 1;
      state.rumbleOperator = 1;
    } else {
      state.rumbleDriver = 0;
      state.rumbleOperator = 0;
    }

    if (state.operatorXPressed) {
      commands.intakeWanted = Intake.State.EJECT;
    }

    if (state.operatorB) {
      if (state.operatorBPressed) {
        state.sourceIntakeStage = Shooter.SourceIntakeStage.INTAKE;
      }
      if (state.sourceIntakeStage == Shooter.SourceIntakeStage.INTAKE && state.pieceInKick) {
        state.sourceIntakeStage = Shooter.SourceIntakeStage.INTAKE_NOTE_DETECTED;
      }
      if (state.sourceIntakeStage == Shooter.SourceIntakeStage.INTAKE_NOTE_DETECTED
          && !state.pieceInKick) {
        state.sourceIntakeStage = Shooter.SourceIntakeStage.HOLD;
      }
      if (state.sourceIntakeStage != Shooter.SourceIntakeStage.HOLD) {
        commands.shooterWanted = Shooter.State.SOURCE;
        commands.kickerWanted = Shooter.KickerState.SOURCE;
        commands.wantedArm = Arm.State.AMP;
        commands.turretWanted = Turret.State.CLIMB;
      }
    }

    boolean shooterReached =
        Util.epsilonEquals(
                state.rightV, state.rightVRef, ShootingConstants.shooterAcceptableErrorRPS)
            && Util.epsilonEquals(
                state.leftV, state.leftVRef, ShootingConstants.shooterAcceptableErrorRPS);

    boolean armReached =
        Util.epsilonEquals(
            state.armPos, state.armPoseRef, ShootingConstants.armAcceptableErrorMotorRotations);

    double turretDegOff =
        Math.abs(
            Units.rotationsToDegrees(state.turretMotorPosition / TurretConstants.gearRatio)
                - Units.rotationsToDegrees(
                    state.turretMotorPositionRef / TurretConstants.gearRatio));
    double metersError = Math.tan(Units.degreesToRadians(turretDegOff)) * state.dist_to_target;

    SmartDashboard.putNumber("Setpoints/Turret Deg Off", turretDegOff);
    SmartDashboard.putNumber("Setpoints/metersError", metersError);

    boolean turretReached =
        Util.epsilonEquals(metersError, 0, ShootingConstants.turretAcceptableErrorMeters);

    if (!ShootingConstants.useTurretMeterError) {
      turretReached =
          Util.epsilonEquals(
              state.turretMotorPosition,
              state.turretMotorPositionRef,
              ShootingConstants.turretAcceptableErrorMotorRotations);
    }

    SmartDashboard.putBoolean("Setpoints/Turret", turretReached);
    SmartDashboard.putBoolean("Setpoints/Arm", armReached);
    SmartDashboard.putBoolean("Setpoints/Shooter", shooterReached);

    if (state.operatorRbPressed) {
      commands.shooterWanted = Shooter.State.SET_SPEED;
      if (shooterReached && armReached && turretReached) {
        commands.wantedLighting = Lighting.State.ALIGNED;
        state.rumbleOperator = 1.0;
      }
    }

    if (state.operatorRtPressed) {
      if (state.armPoseRef > 0.0 && shooterReached && armReached && turretReached) {
        commands.kickerWanted = Shooter.KickerState.KICK;
        commands.wantedLighting = Lighting.State.SHOOTING;
      }
    }

    if (state.operatorDPadLeft) {
      commands.shooterWanted = Shooter.State.HOARD_SHOT;
      commands.wantedArm = Arm.State.HOARD_SHOT;
      commands.turretWanted = Turret.State.HOARD_SHOT;
    }

    if (state.operatorLtPressed) { // AMP
      // Calculate and lock amp turret location - Do not want turret wrapping in amp position
      if (!state.operatorLtLastPressed) {
        commands.ampTurretPosition = Turret.getInstance().snapToAmp(state);
      }
      commands.turretWanted = Turret.State.AMP;
      commands.wantedArm = Arm.State.AMP;
      commands.shooterWanted = Shooter.State.AMP;
    }
    state.operatorLtLastPressed = state.operatorLtPressed;

    /* Climb controls */
    if (state.operatorLbPressed) {
      state.climbMode = true;
    }
    if (state.climbMode) {
      commands.wantedArm =
          state.operatorLbPressed || state.climbUp ? Arm.State.CLIMB : Arm.State.CLIMBED;
      commands.turretWanted = Turret.State.CLIMB;
      commands.shooterWanted = Shooter.State.OFF;
    }
  }

  public void reset(Commands commands) {
    commands.routinesWanted.clear();
    commands.swerveWanted = Swerve.State.NEUTRAL;
    commands.boostWanted = false;
  }
}
