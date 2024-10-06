package com.peninsula.frc2024.config;

import com.peninsula.frc2024.util.input.TunableNumber;
import edu.wpi.first.math.Pair;

public class ShooterConstants {

  public static Pair<Integer, Integer> hoard = Pair.of(70, 60);
  public static TunableNumber shooterLeft = new TunableNumber("Shooter Left", 70);
  public static TunableNumber shooterRight = new TunableNumber("Shooter Right", 70);

  public static Pair<Integer, Integer> max = Pair.of(75, 50);
  public static Pair<Integer, Integer> close = Pair.of(50, 50);
  public static Pair<Integer, Integer> amp = Pair.of(12, 12);
  public static Pair<Integer, Integer> source = Pair.of(-20, -20);

  public static TunableNumber trapVelo = new TunableNumber("lTrapVelo", 25);
  public static TunableNumber hoardVelo = new TunableNumber("lHoardVelo", 40);

  public static Pair<Integer, Integer> trap = Pair.of(-10, 10);

  public static double idlePercentOut = 0.4;

  public static double kickerFastVel = 40.0;
  public static double kickerRunVel = 8.0;
  public static double keepShootingTime = 0.2;
  public static double kickerSourceVel = -10.0;

  /** PID right * */
  public static double kP_right = 0.19438;

  public static double kI_right = 0;
  public static double kD_right = 0;
  public static double kF_right = 0.12813;
  public static double kS_right = 0.047619;

  /** PID left * */
  public static double kP_left = 0.19598;

  public static double kI_left = 0;
  public static double kD_left = 0;
  public static double kF_left = 0.12817;
  public static double kS_left = 0.13459;

  /* Kicker PID */
  public static double kicker_ratio = 18.0 / 30.0;

  public static double kP_kicker = 0.2;
  public static double kI_kicker = 0.0;
  public static double kD_kicker = 0.01;
  public static double kF_kicker = 0.15;
  public static double kS_kicker = 0.08;
}
