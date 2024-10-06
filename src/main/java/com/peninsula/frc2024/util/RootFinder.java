package com.peninsula.frc2024.util;

import java.util.function.Function;

public class RootFinder {

  public static class RootConfiguration {
    public static int itrMax = 20;
    public static double tolerance = .01;
  }

  public static double solve(
      Function<Double, Double> func, Function<Double, Double> fPrime, double x0) {
    double x = x0;

    for (int i = 0; i < RootConfiguration.itrMax; i++) {
      double f = func.apply(x);

      if (Math.abs(f) < RootConfiguration.tolerance) {
        return x;
      }

      double adjustment = f / fPrime.apply(x);
      x -= adjustment;
    }
    return x0;
  }
}
