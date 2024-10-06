package com.peninsula.frc2024.util.peninsulaCoolios;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PolygonField2D extends Polygon2D {

  private final Pose2d[] arrayPose;

  public PolygonField2D(double[] xPoints, double[] yPoints) {
    super(xPoints, yPoints);
    arrayPose = new Pose2d[xPoints.length + 1];

    for (int i = 0; i < xPoints.length; i++) {
      arrayPose[i] = new Pose2d(xPoints[i], yPoints[i], new Rotation2d());
      if (i == 0) arrayPose[xPoints.length] = arrayPose[i];
    }
  }

  public Pose2d[] fieldPoses() {
    return arrayPose;
  }
}
