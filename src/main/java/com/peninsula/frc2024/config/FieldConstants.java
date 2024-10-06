package com.peninsula.frc2024.config;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static Pose2d stageCenter = new Pose2d(new Translation2d(11.69, 4.1), new Rotation2d());

  public static Pose2d ampFSDTargetRed =
      new Pose2d(new Translation2d(14.70564, 7.85), new Rotation2d());
  public static Pose2d ampFSDRedFarTarget =
      new Pose2d(new Translation2d(14.70564, 7.5), new Rotation2d());

  public static double climbFSDDistanceFromCenter = 0.7;
  public static double climbFSDFarDistance = 2.5;

  public static Pose2d climbAmpSideFSDTargetRed =
      distanceFromStageCenter(climbFSDDistanceFromCenter, Math.toRadians(60));
  public static Pose2d climbSourceSideFSDTargetRed =
      distanceFromStageCenter(climbFSDDistanceFromCenter, Math.toRadians(-60));
  public static Pose2d climbFarSideFSDTargetRed =
      distanceFromStageCenter(climbFSDDistanceFromCenter, Math.toRadians(180));

  public static Pose2d climbAmpSideFarRed =
      distanceFromStageCenter(climbFSDFarDistance, Math.toRadians(60));
  public static Pose2d climbSourceSideFarRed =
      distanceFromStageCenter(climbFSDFarDistance, Math.toRadians(-60));
  public static Pose2d climbFarSideFarRed =
      distanceFromStageCenter(climbFSDFarDistance, Math.toRadians(180));

  public static double trapDistance = 1.15;
  public static Pose2d trapAmpSideFSDTargetRed =
      distanceFromStageCenter(trapDistance, Math.toRadians(60));
  public static Pose2d trapSourceSideFSDTargetRed =
      distanceFromStageCenter(trapDistance, Math.toRadians(-60));
  public static Pose2d trapFarSideFSDTargetRed =
      distanceFromStageCenter(trapDistance, Math.toRadians(180));

  public static double fieldLenMeters = Units.inchesToMeters(651.2225);

  private static Pose2d distanceFromStageCenter(double distance, double angle) {
    return stageCenter.transformBy(
        new Transform2d(
            new Translation2d(distance * Math.cos(angle), distance * Math.sin(angle)),
            new Rotation2d(angle)));
  }

  public static class GamePieceLocations {
    public static final double centerlineX = fieldLenMeters / 2.0;
    public static final double blueWingX = Units.inchesToMeters(114);
    public static final double redWingX = fieldLenMeters - Units.inchesToMeters(114);

    public static final double centerline5_Y = Units.inchesToMeters(29.638);
    public static final double centerlineSpacing = Units.inchesToMeters(66);
    public static final double wing1_Y = centerline5_Y + centerlineSpacing * 2;
    public static final double wingSeparationY = Units.inchesToMeters(57);

    public static final Pose2d[] blueWing = new Pose2d[3];
    public static final Pose2d[] redWing = new Pose2d[3];
    public static final Pose2d[] centerline = new Pose2d[5];

    static {
      for (int w = 0; w < 3; w++) {
        blueWing[w] = new Pose2d(blueWingX, wing1_Y - wingSeparationY * w, new Rotation2d());
        redWing[w] = new Pose2d(redWingX, wing1_Y - wingSeparationY * w, new Rotation2d());
      }

      for (int c = 0; c < 5; c++) {
        centerline[c] =
            new Pose2d(centerlineX, centerline5_Y + centerlineSpacing * (4 - c), new Rotation2d());
      }
    }

    public static double blueWingLineX = 5.87;
    public static double redWingLineX = fieldLenMeters - blueWingLineX;

    public static final double sameNoteTolerance = Units.inchesToMeters(8);
    public static final double noteStaleTimeSeconds = 3.0;
    public static final double noteSeeingTimeEpsilon = 0.3;
  }
}
