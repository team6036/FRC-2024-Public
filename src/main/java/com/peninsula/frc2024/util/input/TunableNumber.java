package com.peninsula.frc2024.util.input;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class TunableNumber {
  private ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");
  private GenericEntry num;
  private double defaultVal;

  public TunableNumber(String name, double defaultVal) {
    num = tuningTab.add(name, defaultVal).getEntry();
    num.setDefaultDouble(defaultVal);
    this.defaultVal = defaultVal;
  }

  /**
   * Polls network table for value at that entry.
   *
   * @return default val if nothing there, else value on shuffleboard.
   */
  public double get() {
    return num.getDouble(defaultVal);
  }
}
