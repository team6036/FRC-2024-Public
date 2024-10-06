package com.peninsula.frc2024.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;

/**
 * PerceptionCamera class handles interactions with the NetworkTables for PeninsulaPerception,
 * retrieving observations and transforming them into the robot pose.
 */
public class PerceptionCamera {
  static final String kTableName = "Perception";

  protected final NetworkTable cameraTable;

  DoubleArraySubscriber pose0Entry;
  DoubleArraySubscriber pose1Entry;
  DoubleArraySubscriber errorEntry;
  IntegerArraySubscriber tagsSeenEntry;
  private Transform3d cameraToRobot = new Transform3d();
  private double lastResultReturnedTimestamp;

  private final String tablePath;
  private final String name;

  /**
   * Constructs a new PerceptionCamera using a specified NetworkTableInstance and device name.
   *
   * @param instance The NetworkTableInstance to use for NetworkTables interactions
   * @param deviceName The device id of the perception client
   */
  public PerceptionCamera(NetworkTableInstance instance, String deviceName) {
    name = deviceName;
    var perceptionRootTable = instance.getTable(kTableName);
    this.cameraTable = perceptionRootTable.getSubTable(name);
    tablePath = cameraTable.getPath();

    pose0Entry = cameraTable.getDoubleArrayTopic("observations0").subscribe(null);
    pose1Entry = cameraTable.getDoubleArrayTopic("observations1").subscribe(null);
    errorEntry = cameraTable.getDoubleArrayTopic("errors").subscribe(null);
    tagsSeenEntry = cameraTable.getIntegerArrayTopic("tag_ids").subscribe(new long[0]);
  }

  /**
   * Constructs a new PerceptionCamera using the default NetworkTableInstance and device name.
   *
   * @param deviceName The device id of the perception client
   */
  public PerceptionCamera(String deviceName) {
    this(NetworkTableInstance.getDefault(), deviceName);
  }

  /** Removes all subscribers to NetworkTables */
  public void close() {
    pose0Entry.close();
    pose1Entry.close();
    errorEntry.close();
    tagsSeenEntry.close();
  }

  /**
   * Sets the transform from the camera to the robot
   *
   * @param cameraToRobot The transform to set.
   */
  public void setTransform(Transform3d cameraToRobot) {
    this.cameraToRobot = cameraToRobot;
  }

  /**
   * Retrieves the latest observations from networktable, translated from camera to robot pose.
   *
   * @return An ArrayList of PerceptionObservation objects representing the latest observations
   */
  public ArrayList<PerceptionObservation> getLatestResults() {
    ArrayList<PerceptionObservation> observations = new ArrayList<>();
    try {
      double lastChange = errorEntry.getLastChange() / 1e6;
      if (lastResultReturnedTimestamp == lastChange
          || Timer.getFPGATimestamp() - lastChange > 0.1) {
        return observations;
      }
      double[] pose0Update = pose0Entry.get();
      double[] pose1Update = pose1Entry.get();
      if (pose0Update.length == 7) {
        observations.add(
            new PerceptionObservation(
                observationToPose(pose0Update).transformBy(cameraToRobot),
                errorEntry.get()[0],
                tagsSeenEntry.get(new long[0]).length,
                lastChange));
      } else if (pose1Update.length == 7) {
        observations.add(
            new PerceptionObservation(
                observationToPose(pose1Update).transformBy(cameraToRobot),
                errorEntry.get()[1],
                tagsSeenEntry.get(new long[0]).length,
                lastChange));
      }
      lastResultReturnedTimestamp = errorEntry.getLastChange();
    } catch (ArrayIndexOutOfBoundsException ignored) {
    }
    return observations;
  }

  /**
   * Converts an observation array into a Pose3d object.
   *
   * @param observation The observation array to convert
   * @return A Pose3d object representing the observation
   */
  private Pose3d observationToPose(double[] observation) {
    return new Pose3d(
        new Translation3d(observation[0], observation[1], observation[2]),
        new Rotation3d(
            new Quaternion(observation[3], observation[4], observation[5], observation[6])));
  }
}
