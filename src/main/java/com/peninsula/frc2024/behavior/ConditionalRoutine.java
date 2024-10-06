package com.peninsula.frc2024.behavior;

import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.ReadOnly;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.SubsystemBase;
import java.util.Set;
import java.util.function.Predicate;

public class ConditionalRoutine extends RoutineBase {

  protected RoutineBase mRoutine;
  protected Predicate<RobotState> mPredicate;

  public ConditionalRoutine(RoutineBase routine, Predicate<RobotState> predicate) {
    mRoutine = routine;
    mPredicate = predicate;
  }

  @Override
  protected void update(Commands commands, @ReadOnly RobotState state) {
    mRoutine.execute(commands, state);
  }

  @Override
  protected void stop(Commands commands, @ReadOnly RobotState state) {
    mRoutine.stop(commands, state);
  }

  @Override
  public boolean checkFinished(@ReadOnly RobotState state) {
    return !mPredicate.test(state) || mRoutine.isFinished();
  }

  @Override
  public Set<SubsystemBase> getRequiredSubsystems() {
    return mRoutine.getRequiredSubsystems();
  }
}
