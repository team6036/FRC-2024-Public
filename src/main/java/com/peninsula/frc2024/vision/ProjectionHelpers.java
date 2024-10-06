package com.peninsula.frc2024.vision;

import edu.wpi.first.math.util.Units;
import java.text.DecimalFormat;
import org.ejml.simple.SimpleMatrix;

public class ProjectionHelpers {
  private static final boolean USE_FACTORY_CALIBRATIONS = false;

  // Camera hardware constants
  public static final double CAMERA_ANGLE_DOWN = USE_FACTORY_CALIBRATIONS ? 26.857 : 29.88;
  //      USE_FACTORY_CALIBRATIONS ? 26.857 : 28.417548870401358;
  public static final SimpleMatrix CAMERA_ROTATION =
      new SimpleMatrix(new double[] {CAMERA_ANGLE_DOWN, 0, 0});
  public static final SimpleMatrix CAMERA_TRANSLATION =
      new SimpleMatrix(new double[] {0, Units.inchesToMeters(16.877233), 0.352103});

  // Ring constants
  public static final double HALF_RING_HEIGHT = 0.03;

  /* Camera constants
   * Calibration 3/31/24 10:00 PM (done by Anthony and Daniel at field)
   * [[1036.7211430623324,0.0,642.6887935518934],
   * [0.0,1047.098678722137,505.3146053788958],
   * [0.0,0.0,1.0]]
   * Factory calibration
   * [[1038.54303678,0.0,609.34453296],
   * [0.0,1037.53710926,469.07017464],
   * [0.0,0.0,1.0]]
   */
  public static final double CX = USE_FACTORY_CALIBRATIONS ? 609.34453296 : 642.6887935518934,
      CY = USE_FACTORY_CALIBRATIONS ? 469.07017464 : 505.3146053788958,
      FX = USE_FACTORY_CALIBRATIONS ? 1038.54303678 : 1036.7211430623324,
      FY = USE_FACTORY_CALIBRATIONS ? 1037.53710926 : 1047.098678722137;
  public static final double CAMERA_WIDTH = 1280, CAMERA_HEIGHT = 960;

  public static SimpleMatrix getCameraTranslation(double yRelative) {
    double ringHeight = HALF_RING_HEIGHT + HALF_RING_HEIGHT * yRelative;

    return CAMERA_TRANSLATION.minus(new SimpleMatrix(new double[] {0, 0, ringHeight}));
  }

  public static SimpleMatrix getCameraRotation(double angle) {
    return new SimpleMatrix(new double[] {angle, 0, 0});
  }

  public static SimpleMatrix eulerAnglesToRotationMatrix(SimpleMatrix euler) {
    double roll = -Math.toRadians(euler.get(0));
    double pitch = -Math.toRadians(euler.get(1));
    double yaw = -Math.toRadians(euler.get(2));

    SimpleMatrix R_roll =
        new SimpleMatrix(
            new double[][] {
              {1, 0, 0},
              {0, Math.cos(roll), -Math.sin(roll)},
              {0, Math.sin(roll), Math.cos(roll)}
            });

    SimpleMatrix R_pitch =
        new SimpleMatrix(
            new double[][] {
              {Math.cos(pitch), 0, -Math.sin(pitch)},
              {0, 1, 0},
              {Math.sin(pitch), 0, Math.cos(pitch)}
            });

    SimpleMatrix R_yaw =
        new SimpleMatrix(
            new double[][] {
              {Math.cos(yaw), -Math.sin(yaw), 0},
              {Math.sin(yaw), Math.cos(yaw), 0},
              {0, 0, 1}
            });

    return R_roll.mult(R_pitch).mult(R_yaw);
  }

  public static SimpleMatrix project(int pixel_x, int pixel_y) {
    return project((double) pixel_x, (double) pixel_y);
  }

  public static SimpleMatrix project(int pixel_x, int pixel_y, double angle) {
    return project((double) pixel_x, (double) pixel_y, angle);
  }

  public static SimpleMatrix project(double pixel_x, double pixel_y) {
    return project(pixel_x, pixel_y, CAMERA_ANGLE_DOWN);
  }

  public static SimpleMatrix project(double pixel_x, double pixel_y, double angle) {
    SimpleMatrix cameraDisplacement = getCameraTranslation(pixel_y / CAMERA_HEIGHT);

    // the x and y have different signs because the top left is the origin.
    SimpleMatrix normalized =
        new SimpleMatrix(new double[] {(pixel_x - CX) / FX, 1., (CY - pixel_y) / FY});

    SimpleMatrix cameraRotation = eulerAnglesToRotationMatrix(getCameraRotation(angle));

    SimpleMatrix relativeVector = cameraRotation.mult(normalized);

    double scale = -cameraDisplacement.get(2) / relativeVector.get(2);

    return cameraDisplacement.plus(relativeVector.scale(scale));
  }

  private static final DecimalFormat FORMATTER = new DecimalFormat("#.#####");

  public static String simpleMatrixString(SimpleMatrix matrix) {
    return "(x, y, z) = ("
        + FORMATTER.format(matrix.get(0))
        + ", "
        + FORMATTER.format(matrix.get(1))
        + ", "
        + FORMATTER.format(matrix.get(2))
        + ")";
  }
}
