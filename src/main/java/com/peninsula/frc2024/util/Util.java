package com.peninsula.frc2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** This class holds a bunch of static methods and variables needed for mathematics */
public class Util {

  public static final double kEpsilon = 1e-4;
  public static final double kEpsilon2 = 1e-12;

  private Util() {}

  public static Pose2d newWaypoint(double xInches, double yInches, double yawDegrees) {
    return new Pose2d(
        Units.inchesToMeters(xInches),
        Units.inchesToMeters(yInches),
        Rotation2d.fromDegrees(yawDegrees));
  }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, kEpsilon2);
  }

  /**
   * Applies a deadband to a joystick input value.
   *
   * <p>The deadband is a threshold below which the input value is considered to be zero,
   * eliminating minor unintended movements (jitter). Input values within the deadband range are
   * output as zero. Input values outside the deadband range are scaled from the edge of the
   * deadband to full scale (1 or -1).
   *
   * @param value The raw input value from the joystick to be processed. This value should typically
   *     be between -1.0 and 1.0.
   * @param deadBand The deadband threshold. Values within +/- this threshold will be considered as
   *     zero. The threshold should be a positive value.
   * @return The processed value after applying the deadband. If the absolute value of the input is
   *     less than the threshold, the output will be 0. Otherwise, the output will be scaled from 0
   *     to 1 or -1, depending on the sign of the input.
   */
  public static double handleDeadBand(double value, double deadBand) {
    if (Math.abs(value) < deadBand) {
      return 0.0;
    } else {
      if (value > 0.0) {
        return (value - deadBand) / (1.0 - deadBand);
      } else {
        return (value + deadBand) / (1.0 - deadBand);
      }
    }
  }

  public static double clamp01(double value) {
    return clamp(value, -1.0, 1.0);
  }

  public static double clamp(double value, double minimum, double maximum) {
    return Math.min(maximum, Math.max(minimum, value));
  }

  /**
   * Get the difference in angle between two angles.
   *
   * @param from The first angle
   * @param to The second angle
   * @return The change in angle from the first argument necessary to line up with the second.
   *     Always between -Pi and Pi
   */
  public static double getDifferenceInAngleRadians(double from, double to) {
    return boundAngleNegPiToPiRadians(to - from);
  }

  public static double boundAngleNegPiToPiRadians(double angle) {
    // Naive algorithm
    while (angle >= Math.PI) {
      angle -= 2.0 * Math.PI;
    }
    while (angle < -Math.PI) {
      angle += 2.0 * Math.PI;
    }
    return angle;
  }

  /**
   * Get the difference in angle between two angles.
   *
   * @param from The first angle
   * @param to The second angle
   * @return The change in angle from the first argument necessary to line up with the second.
   *     Always between -180 and 180
   */
  public static double getDifferenceInAngleDegrees(double from, double to) {
    return boundAngleNeg180to180Degrees(to - from);
  }

  public static double boundAngleNeg180to180Degrees(double angle) {
    // Naive algorithm
    while (angle >= 180.0) {
      angle -= 360.0;
    }
    while (angle < -180.0) {
      angle += 360.0;
    }
    return angle;
  }

  public static double boundAngle0to360Degrees(double angle) {
    // Naive algorithm
    while (angle >= 360.0) {
      angle -= 360.0;
    }
    while (angle < 0.0) {
      angle += 360.0;
    }
    return angle;
  }

  public static double boundAngle0to2PiRadians(double angle) {
    // Naive algorithm
    while (angle >= 2.0 * Math.PI) {
      angle -= 2.0 * Math.PI;
    }
    while (angle < 0.0) {
      angle += 2.0 * Math.PI;
    }
    return angle;
  }

  public static boolean approximatelyEqual(double d1, double d2) {
    return withinRange(d1, d2, kEpsilon);
  }

  public static boolean approximatelyEqual(double d1, double d2, double epsilon) {
    return withinRange(d1, d2, epsilon);
  }

  public static boolean withinRange(double d1, double d2, double tolerance) {
    return Math.abs(d1 - d2) < tolerance;
  }

  public static boolean withinRange(
      Pose2d p1, Pose2d p2, double transTolerance, double thetaTolerance) {
    return p1.getTranslation().getDistance(p2.getTranslation()) < transTolerance
        && Math.abs(p1.getRotation().minus(p2.getRotation()).getRadians()) < thetaTolerance;
  }

  public static String classToJsonName(Class<?> clazz) {
    if (clazz.isAnonymousClass()) {
      return "anonymous" + clazz.getSuperclass().getSimpleName();
    }
    String className = clazz.getSimpleName();
    // Make first character lowercase to match JSON conventions
    return Character.toLowerCase(className.charAt(0)) + className.substring(1);
  }

  /**
   * @param a start point
   * @param b end point
   * @param t value between 0 and 1
   * @return lerped value (linear interpolation)
   */
  public static double lerp(double a, double b, double t) {
    return a * (1 - t) + b * t;
  }

  public static Pose2d lerp(Pose2d a, Pose2d b, double t) {
    return new Pose2d(
        lerp(a.getTranslation(), b.getTranslation(), t), lerp(a.getRotation(), b.getRotation(), t));
  }

  public static Translation2d lerp(Translation2d a, Translation2d b, double t) {
    return new Translation2d(lerp(a.getX(), b.getX(), t), lerp(a.getY(), b.getY(), t));
  }

  public static Rotation2d lerp(Rotation2d a, Rotation2d b, double t) {
    return new Rotation2d(lerp(a.getRadians(), b.getRadians(), t));
  }

  /**
   * Calculates the perpendicular distance from a point to a line. Both the point and the line are
   * represented by Pose2d objects.
   *
   * @param pose The Pose2d representing the point.
   * @param pointOnLine The Pose2d representing a point on the line
   * @param lineAngle The direction of the line
   * @return The perpendicular distance from the point to the line.
   */
  public static double calculatePerpendicularDistance(
      Pose2d pose, Pose2d pointOnLine, Rotation2d lineAngle) {
    // Extract positions and rotation
    double x0 = pose.getX();
    double y0 = pose.getY();
    double x1 = pointOnLine.getX();
    double y1 = pointOnLine.getY();

    // Calculate the distance using the sine and cosine of the line's angle
    return Math.abs(
        (x0 - x1) * Math.sin(lineAngle.getRadians())
            - (y0 - y1) * Math.cos(lineAngle.getRadians()));
  }
}
