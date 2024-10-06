package com.peninsula.frc2024.auto.drive;

import com.peninsula.frc2024.auto.AutoBase;
import com.peninsula.frc2024.behavior.RoutineBase;
import com.peninsula.frc2024.behavior.routines.TimedRoutine;

public class None implements AutoBase {
  @Override
  public RoutineBase getRoutine() {
    return new TimedRoutine(1.0);
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
