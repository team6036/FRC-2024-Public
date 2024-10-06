package com.peninsula.frc2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Flipper {
  public static double FL = 16.541;

  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(flipTrans(pose.getTranslation()), flipRot(pose.getRotation()));
  }

  public static Translation2d flipTrans(Translation2d trans) {
    return new Translation2d(FL - trans.getX(), trans.getY());
  }

  public static Rotation2d flipRot(Rotation2d rot) {
    return new Rotation2d(-rot.getCos(), rot.getSin());
  }
}
