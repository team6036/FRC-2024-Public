package com.peninsula.frc2024.behavior;

import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.ReadOnly;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.SubsystemBase;
import java.util.Set;
import java.util.function.Function;

public class FunctionChooseRoutine extends RoutineBase {

  protected RoutineBase mRoutine;
  protected Function<RobotState, RoutineBase> function;
  boolean setup = false;

  public FunctionChooseRoutine(Function<RobotState, RoutineBase> function) {
    this.function = function;
  }

  @Override
  protected void update(Commands commands, @ReadOnly RobotState state) {
    if (!setup) {
      mRoutine = function.apply(state);
      setup = true;
    }
    mRoutine.execute(commands, state);
  }

  @Override
  protected void stop(Commands commands, @ReadOnly RobotState state) {
    mRoutine.stop(commands, state);
  }

  @Override
  public boolean checkFinished(@ReadOnly RobotState state) {
    if (!setup) {
      return false;
    }
    return mRoutine.isFinished();
  }

  @Override
  public Set<SubsystemBase> getRequiredSubsystems() {
    return mRoutine.getRequiredSubsystems();
  }
}
