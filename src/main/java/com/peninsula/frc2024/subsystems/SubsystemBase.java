package com.peninsula.frc2024.subsystems;

import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.ReadOnly;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.util.Util;

public abstract class SubsystemBase {

  private final String mName;

  protected SubsystemBase() {
    mName = Util.classToJsonName(getClass());
  }

  public abstract void update(@ReadOnly Commands commands, @ReadOnly RobotState state);

  @Override
  public String toString() {
    return getName();
  }

  public String getName() {
    return mName;
  }
}
