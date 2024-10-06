package com.peninsula.frc2024.util.peninsulaCoolios;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Util for turrets with a > 360 degree range * */
public class Turretmaxxing {

  public enum WrapStrategy {
    EARLY,
    TRY_KEEP,
    PREDICTIVE
  }

  /**
   * @param wantedAngle wanted angle as a Rotation2d
   * @param maxAngleRotations hardware limit– deviation from 0 angle
   * @param currentAngularVelocity wantedAngle rate in rotations per second
   * @param last_turret_angle current turret angle
   * @param strategy TRY_KEEP will try to keep current direction if possible. PREDICTIVE: prediction
   *     based on future
   * @return a - or + angle in rotations
   */
  public static double optimizeTurretAngle(
      Rotation2d wantedAngle,
      double maxAngleRotations,
      double currentAngularVelocity,
      double last_turret_angle,
      WrapStrategy strategy) {

    double toReturn = wrap5(wantedAngle.getRotations());

    if (toReturn > 1 - maxAngleRotations || toReturn < -1 + maxAngleRotations) {
      // Dual zone
      if (strategy == WrapStrategy.TRY_KEEP) {
        if (last_turret_angle < 0) {
          return (toReturn < 0) ? toReturn : toReturn - 1;
        } else {
          return (toReturn >= 0) ? toReturn : toReturn + 1;
        }
      } else if (strategy == WrapStrategy.PREDICTIVE) {
        double dt = 0.0;

        Rotation2d future = Rotation2d.fromRotations(wrap5(toReturn + dt * currentAngularVelocity));
        double future_rot = future.getRotations();

        if (future_rot > 1 - maxAngleRotations || future_rot < -1 + maxAngleRotations) {
          // Future is in the dual zone so attempt to match future wrap direction
          if (future_rot < 0) {
            return (wantedAngle.getRotations() < 0)
                ? wantedAngle.getRotations()
                : wantedAngle.getRotations() - 1;
          } else {
            return (wantedAngle.getRotations() >= 0)
                ? wantedAngle.getRotations()
                : wantedAngle.getRotations() + 1;
          }
        }

        return optimizeTurretAngle(
            wantedAngle,
            maxAngleRotations,
            currentAngularVelocity,
            last_turret_angle,
            WrapStrategy.TRY_KEEP);
      }
    }

    return toReturn;
  }

  public static double wrap5(double angle) {
    angle %= 1;
    if (angle > 0.5) {
      angle--;
    } else if (angle < -0.5) {
      angle++;
    }
    return angle;
  }

  /**
   * Everything is in field space and meters where possible
   *
   * @param robot_position robot position
   * @param goal_position goal position
   * @param vx robot velocity x component
   * @param vy robot velocity y component
   * @param w robot angular velocity (rotations / second)
   * @return turret angle feedforward (rotations / second)
   */
  public static double getAngularVelocityFeedforward(
      Translation2d robot_position, Translation2d goal_position, double vx, double vy, double w) {

    double lookaheadTime = 0.02;

    Rotation2d currentAngle = angleToTargetFieldRelative(robot_position, goal_position);
    Rotation2d futureAngle =
        futureAngleToTargetFieldRelative(robot_position, goal_position, vx, vy);

    return -w + (futureAngle.getRotations() - currentAngle.getRotations()) / lookaheadTime;
  }

  public static Rotation2d futureAngleToTargetFieldRelative(
      Translation2d robot_position, Translation2d goal_position, double vx, double vy) {

    double lookaheadTime = 0.02;

    return angleToTargetFieldRelative(
        new Translation2d(
            robot_position.getX() + vx * lookaheadTime, robot_position.getY() + vy * lookaheadTime),
        goal_position);
  }

  /**
   * @param robot_position robot position in field space
   * @param goal_position goal position in field space
   * @return field relative angle the shooter needs to point from robot at goal
   */
  public static Rotation2d angleToTargetFieldRelative(
      Translation2d robot_position, Translation2d goal_position) {

    return new Rotation2d(
        goal_position.getX() - robot_position.getX(), goal_position.getY() - robot_position.getY());
  }

  public static Rotation2d robotRelativeFromFieldRelative(
      Rotation2d field_to_robot_rotation, Rotation2d field_to_turret_rotation) {

    return field_to_turret_rotation.minus(field_to_robot_rotation);
  }
}
