package com.peninsula.frc2024.auto;

import com.peninsula.frc2024.behavior.RoutineBase;
import com.peninsula.frc2024.util.Util;

public interface AutoBase {

  RoutineBase getRoutine();

  default String getName() {
    return Util.classToJsonName(getClass());
  }
}
