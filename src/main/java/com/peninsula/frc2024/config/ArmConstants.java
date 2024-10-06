package com.peninsula.frc2024.config;

import com.peninsula.frc2024.util.input.TunableNumber;

public class ArmConstants {

  public static double intake = 0.0;
  public static double amp = 0.34;
  public static double amp_shoot = 0.27;
  public static double source = 0.10;

  public static double climb = 0.31;
  public static double climbed = 0.001;

  public static double hoard_shot = 0.13;

  public static final TunableNumber shooterAngle = new TunableNumber("shooter angle", 0.0);
  public static final TunableNumber hoardNum = new TunableNumber("hoardAngle", hoard_shot);
  public static final TunableNumber trapNum = new TunableNumber("trapAngle", 0.14);
  public static final TunableNumber ampNum = new TunableNumber("ampAngle", amp);

  public static double pivotGearRatio = (54 / 12.0) * (64 / 20.0) * (64 / 8.0);

  /** Big Pivot * */
  public static double kP_big = 4.5;

  public static double kI_big = 0;
  public static double kD_big = 0;
  public static double kF_big = 0;
  public static double kS_big = 0;

  public static double cruiseVelocity_big = 1000;
  public static double cruiseAcceleration_big = 500;
}
