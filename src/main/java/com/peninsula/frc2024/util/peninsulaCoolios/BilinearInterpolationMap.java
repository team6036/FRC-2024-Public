package com.peninsula.frc2024.util.peninsulaCoolios;

import com.peninsula.frc2024.util.Util;

/** https://en.wikipedia.org/wiki/Bilinear_interpolation */
public class BilinearInterpolationMap {

  private final int width, height;
  private final double[][] discreteMap;
  private final int indexToMeters;

  /**
   * @param discreteMap maps index to the discrete value
   * @param indexToMeters index * indexToMeters = meters
   */
  public BilinearInterpolationMap(double[][] discreteMap, int indexToMeters) {

    if (indexToMeters <= 0) throw new RuntimeException("Index to meters must be greater than 0");

    width = discreteMap[0].length;
    height = discreteMap.length;

    this.discreteMap = discreteMap;
    this.indexToMeters = indexToMeters;
  }

  /**
   * @param discreteMap maps index to the discrete value
   */
  public BilinearInterpolationMap(double[][] discreteMap) {
    this(discreteMap, 1);
  }

  /**
   * Does bilinear interpolation
   *
   * @param x going right from leftmost
   * @param y going down from topmost
   */
  public double getIndex(double x, double y) {
    x = Util.clamp(x, 0, width - 1);
    y = Util.clamp(y, 0, height - 1);

    double[][] neighbors = {
      {
        discreteMap[(int) Math.floor(y)][(int) Math.floor(x)],
        discreteMap[(int) Math.floor(y)][(int) Math.ceil(x)]
      },
      {
        discreteMap[(int) Math.ceil(y)][(int) Math.floor(x)],
        discreteMap[(int) Math.ceil(y)][(int) Math.ceil(x)]
      }
    };

    // Bilinear interpolation strategy
    // [0, 0]   [0, 1]
    //   |        |
    //   k1 – r – k2
    //   |        |
    // [1, 0]   [1, 1]

    double k1 = neighbors[0][0] + (neighbors[1][0] - neighbors[0][0]) * (y - Math.floor(y));
    double k2 = neighbors[0][1] + (neighbors[1][1] - neighbors[0][1]) * (y - Math.floor(y));

    return k1 + (k2 - k1) * (x - Math.floor(x));
  }

  public double getMeters(double x, double y) {
    return getIndex(x / indexToMeters, y / indexToMeters);
  }
}
