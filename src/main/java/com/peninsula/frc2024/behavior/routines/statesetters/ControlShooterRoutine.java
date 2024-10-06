package com.peninsula.frc2024.behavior.routines.statesetters;

import com.peninsula.frc2024.behavior.TimeoutRoutineBase;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.ReadOnly;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.Shooter;

public class ControlShooterRoutine extends TimeoutRoutineBase {

  Shooter.State wantedState;

  public ControlShooterRoutine(Shooter.State state, double timeout) {
    super(timeout);
    wantedState = state;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    commands.shooterWanted = wantedState;
  }

  @Override
  public boolean checkIfFinishedEarly(RobotState state) {
    return false;
  }
}
