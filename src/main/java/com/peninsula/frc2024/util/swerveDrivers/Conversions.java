package com.peninsula.frc2024.util.swerveDrivers;

public class Conversions {

  /**
   * @param counts Falcon Counts
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Degrees of Rotation of Mechanism
   */
  public static double falconToDegrees(double counts, double gearRatio) {
    return (counts * 360.0) / gearRatio;
  }

  /**
   * @param degrees Degrees of rotation of Mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Falcon Counts
   */
  public static double degreesToFalcon(double degrees, double gearRatio) {
    return (degrees * gearRatio) / 360.0;
  }

  /**
   * @param velocityCounts Falcon Velocity Counts
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return RPM of Mechanism
   */
  public static double falconToRPM(double velocityCounts, double gearRatio) {
    return velocityCounts / gearRatio;
  }

  /**
   * @param RPM RPM of mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return RPM of Mechanism
   */
  public static double RPMToFalcon(double RPM, double gearRatio) {
    return RPM * gearRatio;
  }

  /**
   * @param velocityCounts Falcon Velocity Counts
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Falcon Velocity Counts
   */
  public static double falconToMPS(double velocityCounts, double circumference, double gearRatio) {
    double wheelMPS = (velocityCounts / gearRatio * circumference);
    return wheelMPS;
  }

  /**
   * @param velocity Velocity MPS
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Falcon Velocity Counts
   */
  public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
    double wheelRPM = ((velocity * 60) / circumference);
    double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
    return wheelVelocity;
  }

  // Convert meters to inches
  public static double metersToInches(double meters) {
    return meters * (39.73701 / 1);
  }

  // Convert meters to inches
  public static double inchesToMeters(double inches) {
    return inches * (0.0254 / 1);
  }

  /**
   * Converts meters per second to rotations per second
   *
   * @param velocity Velocity in meters per second
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Falcon velocity in rotations per second
   */
  public static double MPSToRPS(double velocity, double circumference, double gearRatio) {
    double wheelRPS = (velocity / circumference);
    double wheelVelocity = wheelRPS * gearRatio;
    return wheelVelocity;
  }

  public static double RPSToMPS(double velocity, double circumference, double gearRatio) {
    return velocity * circumference / gearRatio;
  }
}
