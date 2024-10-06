package com.peninsula.frc2024.behavior.routines.statesetters;

import com.peninsula.frc2024.behavior.TimeoutRoutineBase;
import com.peninsula.frc2024.config.ShooterConstants;
import com.peninsula.frc2024.config.ShootingConstants;
import com.peninsula.frc2024.config.TurretConstants;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.ReadOnly;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.*;
import com.peninsula.frc2024.util.Util;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ContinuousIntakeShootRoutine extends TimeoutRoutineBase {

  public ContinuousIntakeShootRoutine(double timeout) {
    super(timeout);
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    commands.turretWanted = Turret.State.AIM_SHOOT_ON_MOVE;
    commands.wantedArm = Arm.State.SHOOTING_AIM;
    commands.shooterWanted = Shooter.State.SET_SPEED;
    commands.kickerWanted = Shooter.KickerState.HOLD;
    commands.intakeWanted = Intake.State.OFF;

    if (state.pieceInKick) {
      commands.wantedLighting = Lighting.State.HAS_NOTE;
    } else {
      commands.wantedLighting = Lighting.State.IDLE;

      if (state.gameTimeS - state.pieceLeftTime > ShooterConstants.keepShootingTime) {
        commands.turretWanted = Turret.State.INTAKE;
        commands.wantedArm = Arm.State.INTAKE;
      }
    }

    /* Intake controls */
    if (!state.pieceInKick) {
      state.climbMode = false;
      if (Math.abs(state.turretMotorPosition) < ShootingConstants.maxIntakeRunTurretError) {
        commands.intakeWanted = Intake.State.RUN;
      }
      commands.turretWanted = Turret.State.INTAKE;
      commands.wantedArm = Arm.State.INTAKE;
      commands.wantedLighting = Lighting.State.INTAKING;
    }
    if (state.pieceInKick) {
      commands.wantedLighting = Lighting.State.HAS_NOTE;
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

    boolean turretReached =
        Util.epsilonEquals(metersError, 0, ShootingConstants.turretAcceptableErrorMeters);

    SmartDashboard.putBoolean("Setpoints/Turret", turretReached);
    SmartDashboard.putBoolean("Setpoints/Arm", armReached);
    SmartDashboard.putBoolean("Setpoints/Shooter", shooterReached);
    SmartDashboard.putNumber("Setpoints/Meters Error", metersError);

    if (!ShootingConstants.useTurretMeterError) {
      turretReached =
          Util.epsilonEquals(
              state.turretMotorPosition,
              state.turretMotorPositionRef,
              ShootingConstants.turretAcceptableErrorMotorRotations);
    }

    if (shooterReached && armReached && turretReached) {
      commands.kickerWanted = Shooter.KickerState.KICK;
      commands.wantedLighting = Lighting.State.SHOOTING;
    }
  }

  @Override
  public boolean checkIfFinishedEarly(RobotState state) {
    return false;
  }
}
