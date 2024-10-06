package com.peninsula.frc2024.config;

import com.peninsula.frc2024.util.ShotMap;

public class ShootingConstants {
  public static double k = 0.075;

  /** Acceptable error * */
  public static double shooterAcceptableErrorRPS = 3;

  public static double armAcceptableErrorMotorRotations = 0.3;

  public static boolean useTurretMeterError = true;

  public static double turretAcceptableErrorMotorRotations = 1;
  public static double turretAcceptableErrorMeters = 0.2;

  public static double maxIntakeRunTurretError = 0.2 * TurretConstants.gearRatio;

  public static double turretAngleOffset = 2 / 360.0;
  public static double hoardTurretAngleOffset = 12 / 360.0;

  /** Amp * */
  public static boolean ampDirect = true; // bounce into

  /** Tuning * */
  public static ShotMap shooterMap = new ShotMap();

  static {
    shooterMap.put(0.0, 0.18, 75, 50);
    shooterMap.put(1.02, 0.143, 75, 50);
    shooterMap.put(1.267, 0.13, 75, 50);
    shooterMap.put(1.48, 0.12, 75, 50);
    shooterMap.put(1.75, 0.091, 75, 50);
    shooterMap.put(2.26, 0.077, 75, 50);
    shooterMap.put(2.77, 0.065, 75, 50);
    shooterMap.put(3.26, 0.057, 75, 50);
    shooterMap.put(3.75, 0.0485, 75, 50);
    shooterMap.put(4.25, 0.043, 75, 50);
    shooterMap.put(4.75, 0.038, 75, 50);
    shooterMap.put(5.28, 0.035, 75, 50);
    shooterMap.put(5.75, 0.0325, 75, 50);
    shooterMap.put(6.25, 0.032, 75, 50);
    shooterMap.put(6.75, 0.0315, 75, 50);
  }

  public static ShotMap hoardMap = new ShotMap();

  static {
    hoardMap.put(10.668, 0.11, 58, 38.7);
    hoardMap.put(9.144, 0.12, 55, 36.7);
    hoardMap.put(7.62, 0.13, 52, 34.7);
    hoardMap.put(6.096, 0.14, 45, 30);
    hoardMap.put(4.6, 0.14, 35, 23.3);
    hoardMap.put(4.572, 0.001, 40, 26.7);
  }
}
