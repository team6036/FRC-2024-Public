package com.peninsula.frc2024.vision;

import edu.wpi.first.networktables.*;
import java.util.Set;

/**
 * The PerceptionConfigurator class handles the transmission of global configuration data to vision
 * coprocessors via NetworkTables. This shared configuration is utilized by all the connected
 * coprocessors to maintain consistent behavior in vision processing.
 */
public class PerceptionConfigurator {
  static final String kTableName = "Perception";

  protected final NetworkTable configTable;

  IntegerArrayPublisher ignoredTagsEntry;

  /** Constructs a PerceptionConfigurator with the default NetworkTableInstance. */
  public PerceptionConfigurator() {
    this(NetworkTableInstance.getDefault());
  }

  /**
   * Constructs a PerceptionConfigurator with a specified NetworkTableInstance.
   *
   * @param instance The NetworkTableInstance to use for configuration.
   */
  public PerceptionConfigurator(NetworkTableInstance instance) {
    configTable = instance.getTable(kTableName).getSubTable("config");
    ignoredTagsEntry = configTable.getIntegerArrayTopic("ignored_tags").publish();

    setIgnoredTags(Set.of());
  }

  /**
   * Sets the list of tag IDs to be ignored. This applies to all cameras
   *
   * @param ignoredTags An ArrayList of Integers representing the tags to be ignored.
   */
  public void setIgnoredTags(Set<Integer> ignoredTags) {
    ignoredTagsEntry.set(ignoredTags.stream().mapToLong(Integer::longValue).toArray());
  }
}
