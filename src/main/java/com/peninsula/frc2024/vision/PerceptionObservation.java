package com.peninsula.frc2024.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Represents an observation made by the PerceptionCamera, including the pose, error, number of tags
 * seen, and a timestamp for when the observation was made.
 */
public class PerceptionObservation {
  private static final double angleDifferenceStdMultiplier = 0.01;
  private static final double numTagStdMultiplier = 1.0;
  private static final double reprojErrorStdMultiplier = 0.0;
  private static final double xyStdDevCoefficient = 0.01;
  private static final double thetaStdDevCoefficient = 0.005;

  private final double error;
  private final int numTags;
  private final Pose3d pose;
  private final double timestampSeconds;

  /**
   * Constructs a PerceptionObservation with a given pose, error value, number of tags seen, and
   * timestamp.
   *
   * @param pose The pose of the robot observed by the camera.
   * @param error The reprojection error value associated with the observation.
   * @param numTags The number of tags seen in the observation.
   * @param timestamp The time at which the observation was made, in seconds.
   */
  public PerceptionObservation(Pose3d pose, double error, int numTags, double timestamp) {
    this.pose = pose;
    this.error = error;
    this.numTags = numTags;
    this.timestampSeconds = timestamp;
  }

  /**
   * Constructs a default PerceptionObservation with no tags seen, maximum error, and a default
   * pose.
   */
  public PerceptionObservation() {
    this(new Pose3d(), Integer.MAX_VALUE, 0, -1);
  }

  /**
   * Retrieves the pose associated with the observation.
   *
   * @return A Pose3d object representing the robot pose
   */
  public Pose3d getPose() {
    return pose;
  }

  public Pose2d getPose2d() {
    return pose.toPose2d();
  }

  /**
   * Retrieves the number of tags seen in the observation.
   *
   * @return The number of tags seen
   */
  public int getNumTags() {
    return numTags;
  }

  /**
   * Retrieves the reprojection error of the observation.
   *
   * @return The reprojection error
   */
  public double getError() {
    return error;
  }

  public Matrix<N3, N1> getStdMatrix(Rotation2d gyroAngle) {

    // No tags - just in case
    if (numTags == 0) {
      return VecBuilder.fill(Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE);
    }

    double angleDiff = Math.abs(gyroAngle.minus(pose.toPose2d().getRotation()).getDegrees());

    double standardDeviation =
        numTagStdMultiplier
            / Math.pow(numTags, 2)
            * Math.pow(angleDiff * angleDifferenceStdMultiplier, 3);

    double xyStd = xyStdDevCoefficient * standardDeviation;
    double thetaStd = thetaStdDevCoefficient * standardDeviation;

    return VecBuilder.fill(xyStd, xyStd, thetaStd);
  }

  /**
   * Retrieves the timestamp of when the observation was made.
   *
   * @return The timestamp in seconds
   */
  public double getTimestampSeconds() {
    return timestampSeconds;
  }
}
