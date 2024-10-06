package com.peninsula.frc2024.util.peninsulaCoolios;

/** Stores a polygon as an array of double coordinates. */
public class Polygon2D {
  private final double[] xPoints, yPoints;
  private final int n;

  public Polygon2D(double[] xPoints, double[] yPoints) {

    this.xPoints = xPoints;
    this.yPoints = yPoints;

    n = xPoints.length;
  }

  /**
   * Casts a ray to the right and checks number of intersections with the polygon. If this value is
   * odd, the point inside and if its even then its outside.
   * https://en.wikipedia.org/wiki/Point_in_polygon
   *
   * @param x coordinate to check x value
   * @param y coordinate to check y value
   * @return if the coordinate in within the polygon
   */
  public boolean PIP(double x, double y) {

    int intersections = 0;

    for (int i = 0; i < n; i++) {
      double x1 = xPoints[i], y1 = yPoints[i];
      double x2 = xPoints[(i + 1) % n], y2 = yPoints[(i + 1) % n];

      if (y1 > y2) {
        double xTemp = x1;
        double yTemp = y1;
        x1 = x2;
        x2 = xTemp;
        y1 = y2;
        y2 = yTemp;
      }

      double xInterpolated = x1 + (x2 - x1) * ((y - y1) / (y2 - y1));

      if (xInterpolated == x) return true; // On the line

      if (xInterpolated >= x && y <= y2 && y >= y1) intersections++;
    }
    return (intersections & 1) == 1;
  }
}
