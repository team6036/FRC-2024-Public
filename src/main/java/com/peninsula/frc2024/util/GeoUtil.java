package com.peninsula.frc2024.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class GeoUtil {
  public static final Twist2d identidyTwist = new Twist2d(0, 0, 0);
  public static final Rotation2d kPi = new Rotation2d(Math.PI);

  public static boolean approximatelyEqual(Twist2d a, Twist2d b, double epsilon) {
    return withinRange(a.dx, b.dx, epsilon)
        && withinRange(a.dy, b.dy, epsilon)
        && withinRange(a.dtheta, b.dtheta, epsilon);
  }

  public static boolean approximatelyEqual(Twist2d a, Twist2d b) {
    return approximatelyEqual(a, b, Util.kEpsilon);
  }

  public static Twist2d ChassisSpeedToTwist2d(ChassisSpeeds speed) {
    return new Twist2d(
        speed.vxMetersPerSecond, speed.vyMetersPerSecond, speed.omegaRadiansPerSecond);
  }

  public static boolean withinRange(double d1, double d2, double tolerance) {
    return Math.abs(d1 - d2) < tolerance;
  }

  /**
   * Flips the rotation of a rotation 2d. For example,
   *
   * @param angle
   * @return
   */
  public static Rotation2d flipRotation2d(Rotation2d angle) {
    return new Rotation2d(angle.getRadians() + Math.PI);
  }

  /**
   * The inverse of a Rotation2d "undoes" the effect of this rotation.
   *
   * @return The inverse of this rotation.
   */
  public static Rotation2d inverseRotation2d(Rotation2d angle) {
    return Rotation2d.fromRadians(-angle.getRadians());
  }
}
