package com.peninsula.frc2024.behavior.routines.statesetters;

import com.peninsula.frc2024.behavior.TimeoutRoutineBase;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.ReadOnly;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.Turret;

public class ControlTurretRoutine extends TimeoutRoutineBase {

  Turret.State wantedState;

  public ControlTurretRoutine(Turret.State state, double timeout) {
    super(timeout);
    wantedState = state;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    commands.turretWanted = wantedState;
  }

  @Override
  public boolean checkIfFinishedEarly(RobotState state) {
    return false;
  }
}
