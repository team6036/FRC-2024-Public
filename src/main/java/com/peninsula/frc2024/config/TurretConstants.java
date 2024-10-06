package com.peninsula.frc2024.config;

public class TurretConstants {

  public static double gearRatio = (64.0 / 14) * (60.0 / 20.0) * (140.0 / 44);
  public static boolean motorInverted = true;
  //  public static boolean motorInverted = true;

  public static double encoderGearRatio = (140.0 / 44);
  public static double encoderMagnetOffset = -0.4639;
  public static boolean encoderInverted = false;

  public static double intakeSetpointRot = 0;
  public static double ampSetpointRot = 0.25;
  public static double climbSetpointRot = 0.5;
  public static double trapSetpointRot = 0.48;

  public static double maxDeviationRotations = 0.8;

  public static boolean enableSoftLimits = false;

  /** PID right * */
  public static double kP = 1.4;

  public static double kI = 0;
  public static double kD = 0.06;
  public static double kF = 1.0 / (6000 / 60.0);
  public static double kS = 0.5;

  public static double mmAcceleration = 2500;
  public static double mmCruiseVelocity = 2000;
}
