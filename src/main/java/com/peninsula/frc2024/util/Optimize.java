package com.peninsula.frc2024.util;

import com.peninsula.frc2024.config.ShootingConstants;
import com.peninsula.frc2024.config.VisionConstants;

public class Optimize {
  public static double H = VisionConstants.goal_position_red.getZ();

  private static double t(double d) {
    return ShootingConstants.k * Math.sqrt(H * H + d * d);
  }

  public static double error(
      double x, double y, double gx, double gy, double vx, double vy, double d) {
    return Math.pow((x - gx + vx * t(d)), 2) + Math.pow(y - gy + vy * t(d), 2) - d * d;
  }

  public static double errorDerivative(
      double x, double y, double gx, double gy, double vx, double vy, double d) {
    return 2
            * d
            * ShootingConstants.k
            * vx
            * (-gx + x + ShootingConstants.k * vx * Math.sqrt(H * H + d * d))
            / (Math.sqrt(H * H + d * d))
        + 2
            * d
            * ShootingConstants.k
            * vy
            * (-gy + y + ShootingConstants.k * vy * Math.sqrt(H * H + d * d))
            / (Math.sqrt(H * H + d * d))
        - 2 * d;
  }

  public static double getShotDistanceCrescendo(
      double x, double y, double vx, double vy, double gx, double gy) {

    double dGuess = Math.hypot(x - gx, y - gy);

    return RootFinder.solve(
        (d) -> error(x, y, gx, gy, vx, vy, d),
        (d) -> errorDerivative(x, y, gx, gy, vx, vy, d),
        dGuess);
  }

  public static double getTOF(double x, double y, double vx, double vy, double gx, double gy) {
    double dist = getShotDistanceCrescendo(x, y, vx, vy, gx, gy);

    return Math.sqrt(H * H + dist * dist) * ShootingConstants.k;
  }
}
