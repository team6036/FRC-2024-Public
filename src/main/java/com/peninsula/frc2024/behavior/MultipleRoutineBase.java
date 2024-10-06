package com.peninsula.frc2024.behavior;

import java.util.List;

/** Common base class for routines that manage a list of children routines. */
public abstract class MultipleRoutineBase extends RoutineBase {

  protected final List<RoutineBase> mRoutines;

  public MultipleRoutineBase(RoutineBase... routines) {
    this(List.of(routines));
  }

  public MultipleRoutineBase(List<RoutineBase> routines) {
    mRoutines = routines;
  }

  public List<RoutineBase> getRoutines() {
    return mRoutines;
  }

  @Override
  public String toString() {
    var status = new StringBuilder(super.getName()).append(":");
    for (RoutineBase routine : mRoutines) {
      status
          .append("\n")
          .append("    ")
          .append(routine)
          .append(" ")
          .append("[")
          .append(routine.getStatus())
          .append("]");
    }
    return status.toString();
  }
}
