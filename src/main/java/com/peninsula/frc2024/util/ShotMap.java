package com.peninsula.frc2024.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShotMap {
  public InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
  public InterpolatingDoubleTreeMap rpmLeftMap = new InterpolatingDoubleTreeMap();
  public InterpolatingDoubleTreeMap rpmRightMap = new InterpolatingDoubleTreeMap();

  public void put(double key, double angle, double rpmLeft, double rpmRight) {
    angleMap.put(key, angle);
    rpmLeftMap.put(key, rpmLeft);
    rpmRightMap.put(key, rpmRight);
  }

  public ShotData get(double key) {
    return new ShotData(angleMap.get(key), rpmLeftMap.get(key), rpmRightMap.get(key));
  }

  public record ShotData(double angle, double rpmLeft, double rpmRight) {}
}
