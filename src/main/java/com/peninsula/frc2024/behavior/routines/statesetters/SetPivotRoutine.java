package com.peninsula.frc2024.behavior.routines.statesetters;

import com.peninsula.frc2024.behavior.TimeoutRoutineBase;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.ReadOnly;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.Arm;

public class SetPivotRoutine extends TimeoutRoutineBase {

  Arm.State wantedState;

  public SetPivotRoutine(Arm.State state, double timeout) {
    super(timeout);
    wantedState = state;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    commands.wantedArm = wantedState;
  }

  @Override
  public boolean checkIfFinishedEarly(RobotState state) {
    return false;
  }
}
