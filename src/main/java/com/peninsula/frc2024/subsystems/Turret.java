package com.peninsula.frc2024.subsystems;

import com.peninsula.frc2024.config.ShootingConstants;
import com.peninsula.frc2024.config.TurretConstants;
import com.peninsula.frc2024.config.VisionConstants;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.Robot;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.util.Util;
import com.peninsula.frc2024.util.control.ControllerOutput;
import com.peninsula.frc2024.util.control.ProfiledGains;
import com.peninsula.frc2024.util.peninsulaCoolios.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Turret extends SubsystemBase {
  public enum State {
    INTAKE,
    AMP,
    AIM_DIRECT,
    AIM_SHOOT_ON_MOVE,
    CLIMB,
    TRAP,
    HOARD_SHOT
  }

  private final ControllerOutput motorOutputs = new ControllerOutput();

  private static final Turret sInstance = new Turret();

  public static Turret getInstance() {
    return sInstance;
  }

  public ControllerOutput getMotorOutputs() {
    return motorOutputs;
  }

  public void update(Commands commands, RobotState state) {
    switch (commands.turretWanted) {
      case AIM_DIRECT:
        motorOutputs.setTargetPositionProfiled(
            directAimAngle(state) * TurretConstants.gearRatio, new ProfiledGains());
        break;
      case AIM_SHOOT_ON_MOVE:
        double ffV =
            Turretmaxxing.getAngularVelocityFeedforward(
                state.lastEst.getTranslation(),
                state.virtual_goal.getTranslation(),
                state.fieldRelativeSpeeds.vxMetersPerSecond,
                state.fieldRelativeSpeeds.vyMetersPerSecond,
                Units.radiansToRotations(state.fieldRelativeSpeeds.omegaRadiansPerSecond));

        double vRPS = (12.0 / (5800 / 60.0)) * TurretConstants.gearRatio;

        motorOutputs.setTargetPositionProfiled(
            (sotmAimAngle(state) + ShootingConstants.turretAngleOffset) * TurretConstants.gearRatio,
            ffV * vRPS,
            new ProfiledGains());
        break;
      case INTAKE:
        motorOutputs.setTargetPositionProfiled(
            TurretConstants.intakeSetpointRot, new ProfiledGains());
        break;
      case AMP:
        double setpoint = commands.ampTurretPosition;
        motorOutputs.setTargetPositionProfiled(
            aimAtAngle(state, Rotation2d.fromRotations(setpoint)) * TurretConstants.gearRatio,
            new ProfiledGains());
        break;
      case CLIMB:
        motorOutputs.setTargetPositionProfiled(
            TurretConstants.climbSetpointRot * TurretConstants.gearRatio, new ProfiledGains());
        break;
      case TRAP:
        motorOutputs.setTargetPositionProfiled(
            TurretConstants.trapSetpointRot * TurretConstants.gearRatio, new ProfiledGains());
        break;
      case HOARD_SHOT:
        ffV =
            Turretmaxxing.getAngularVelocityFeedforward(
                state.lastEst.getTranslation(),
                state.hoard_virtual_goal.getTranslation(),
                state.fieldRelativeSpeeds.vxMetersPerSecond,
                state.fieldRelativeSpeeds.vyMetersPerSecond,
                Units.radiansToRotations(state.fieldRelativeSpeeds.omegaRadiansPerSecond));

        vRPS = (12.0 / (5800 / 60.0)) * TurretConstants.gearRatio;

        motorOutputs.setTargetPositionProfiled(
            (sotmAimAngleHoard(state) + ShootingConstants.hoardTurretAngleOffset)
                * TurretConstants.gearRatio,
            ffV * vRPS,
            new ProfiledGains());
        break;
    }
  }

  public double directAimAngle(RobotState state) {

    Translation2d goal_pos;

    if (Robot.onBlueAlliance)
      goal_pos =
          new Translation2d(
              VisionConstants.goal_position_blue.getX(), VisionConstants.goal_position_blue.getY());
    else
      goal_pos =
          new Translation2d(
              VisionConstants.goal_position_red.getX(), VisionConstants.goal_position_red.getY());

    return Util.clamp(
        shootAtPoint(state, goal_pos),
        -TurretConstants.maxDeviationRotations,
        TurretConstants.maxDeviationRotations);
  }

  public double sotmAimAngle(RobotState state) {
    return shootAtPoint(state, state.virtual_goal.getTranslation());
  }

  public double sotmAimAngleHoard(RobotState state) {
    return shootAtPoint(state, state.hoard_virtual_goal.getTranslation());
  }

  public double shootAtPoint(RobotState state, Translation2d goal_pos) {
    Rotation2d turret_robot_relative_ghost =
        Turretmaxxing.robotRelativeFromFieldRelative(
            state.lastEstAngle,
            Turretmaxxing.angleToTargetFieldRelative(state.lastEst.getTranslation(), goal_pos));

    return Turretmaxxing.optimizeTurretAngle(
        turret_robot_relative_ghost,
        TurretConstants.maxDeviationRotations,
        -Units.radiansToRotations(state.chassisRelativeSpeeds.omegaRadiansPerSecond),
        state.turretMotorPosition,
        Turretmaxxing.WrapStrategy.TRY_KEEP);
  }

  public double aimAtAngle(RobotState state, Rotation2d angle) {
    return Turretmaxxing.optimizeTurretAngle(
        angle,
        TurretConstants.maxDeviationRotations,
        -Units.radiansToRotations(state.chassisRelativeSpeeds.omegaRadiansPerSecond),
        state.turretMotorPosition,
        Turretmaxxing.WrapStrategy.TRY_KEEP);
  }

  /**
   * Snaps the robot's turret to the nearest quarter rotation facing the amp
   *
   * @param state The current state of the robot
   * @return The snapped angle in rotations.
   */
  public double snapToAmp(RobotState state) {
    double angle =
        Turretmaxxing.robotRelativeFromFieldRelative(
                state.lastEstAngle, Rotation2d.fromDegrees(-90))
            .getRotations();
    double roundedAngle = Math.round(angle * 4) / 4.0;

    if (roundedAngle == 0) {
      // Check if the original angle is closer to 0.25 or -0.25 and adjust accordingly
      double distanceToQuarter = Math.abs(angle - 0.25);
      double distanceToMinusQuarter = Math.abs(angle + 0.25);
      if (distanceToQuarter < distanceToMinusQuarter) {
        roundedAngle = 0.25;
      } else {
        roundedAngle = -0.25;
      }
    }

    return roundedAngle;
  }
}
