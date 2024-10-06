package com.peninsula.frc2024.util.peninsulaCoolios.phronesis.fieldAwareness;

public class FieldNoteSim {
  public double[] disappearTimes = new double[] {1d, 1d, 1000d, 1000d, 1000d};

  public boolean isGone(int cNote, double time) {
    return disappearTimes[cNote - 1] <= time;
  }
}
