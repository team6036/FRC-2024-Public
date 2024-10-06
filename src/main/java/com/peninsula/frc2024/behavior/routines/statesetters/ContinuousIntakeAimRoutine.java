package com.peninsula.frc2024.behavior.routines.statesetters;

import com.peninsula.frc2024.behavior.TimeoutRoutineBase;
import com.peninsula.frc2024.config.ShooterConstants;
import com.peninsula.frc2024.config.ShootingConstants;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.ReadOnly;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.*;

public class ContinuousIntakeAimRoutine extends TimeoutRoutineBase {
  public ContinuousIntakeAimRoutine(double timeout) {
    super(timeout);
  }

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
  }

  public boolean checkIfFinishedEarly(RobotState state) {
    return false;
  }
}
