package com.peninsula.frc2024.behavior.routines.statesetters;

import com.peninsula.frc2024.behavior.TimeoutRoutineBase;
import com.peninsula.frc2024.config.ShootingConstants;
import com.peninsula.frc2024.config.TurretConstants;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.ReadOnly;
import com.peninsula.frc2024.robot.Robot;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.*;
import com.peninsula.frc2024.util.Util;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootWhenReadyRoutine extends TimeoutRoutineBase {
  private final double maxWaitTime;
  private double addTrim = 0.0;

  public ShootWhenReadyRoutine(double maxWaitTime, double timeout, double addTrim) {
    super(timeout);
    this.maxWaitTime = maxWaitTime;
    this.addTrim = addTrim;
  }

  public ShootWhenReadyRoutine(double maxWaitTime, double timeout) {
    super(timeout);
    this.maxWaitTime = maxWaitTime;
  }

  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    commands.turretWanted = Turret.State.AIM_SHOOT_ON_MOVE;
    commands.wantedArm = Arm.State.SHOOTING_AIM;
    commands.shooterWanted = Shooter.State.SET_SPEED;
    commands.kickerWanted = Shooter.KickerState.HOLD;
    commands.intakeWanted = Intake.State.OFF;
    state.shooterTrim += addTrim;
    if (mTimer.hasElapsed(maxWaitTime)) {
      commands.kickerWanted = Shooter.KickerState.KICK;
    }

    boolean shooterReached =
        Util.epsilonEquals(state.rightV, state.rightVRef, 6)
            && Util.epsilonEquals(state.leftV, state.leftVRef, 6);

    boolean armReached =
        Util.epsilonEquals(
            state.armPos, state.armPoseRef, ShootingConstants.armAcceptableErrorMotorRotations);

    double turretDegOff =
        Math.abs(
            Units.rotationsToDegrees(state.turretMotorPosition / TurretConstants.gearRatio)
                - Units.rotationsToDegrees(
                    state.turretMotorPositionRef / TurretConstants.gearRatio));
    double metersError = Math.tan(Units.degreesToRadians(turretDegOff)) * state.dist_to_target;

    boolean turretReached = Util.epsilonEquals(metersError, 0, 0.4);

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

  public boolean checkIfFinishedEarly(RobotState state) {
    if (Robot.isRobotReal()) {
      return !state.pieceInKick;
    } else {
      return false;
    }
  }
}
