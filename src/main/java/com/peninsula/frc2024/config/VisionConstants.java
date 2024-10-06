package com.peninsula.frc2024.config;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

@SuppressWarnings("java:S1104")
public class VisionConstants {

  public static final Transform3d robot_to_camera_back_right =
      new Transform3d(
          new Translation3d(-0.431834036, -0.2834393874, 0.2650027212),
          new Rotation3d(new Quaternion(0.290, 0.247, -0.078, 0.921)));

  public static final Transform3d robot_to_camera_back_left =
      new Transform3d(
          new Translation3d(-0.431834036, 0.2834393874, 0.2650027212),
          new Rotation3d(new Quaternion(0.290, -0.247, -0.078, -0.921)));

  public static final Transform3d robot_to_camera_front_left =
      new Transform3d(
          new Translation3d(0.259, 0.260, 0.198),
          new Rotation3d(new Quaternion(0.906, 0.094, -0.277, -0.307)));

  public static final Transform3d robot_to_camera_front_right =
      new Transform3d(
          new Translation3d(0.259, -0.260, 0.198),
          new Rotation3d(new Quaternion(0.906, -0.094, -0.277, 0.307)));

  /* goal positions */
  public static Translation3d goal_position_blue =
      new Translation3d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(218.478069),
          Units.inchesToMeters(80.917132));
  public static Translation3d goal_position_red =
      new Translation3d(
          Units.inchesToMeters(651.2225),
          Units.inchesToMeters(218.478069),
          Units.inchesToMeters(80.917132));

  public static Translation2d hoard_position_blue = new Translation2d(1, 7);
  public static Translation2d hoard_position_red =
      new Translation2d(FieldConstants.fieldLenMeters - 1, 7);
}
