package com.peninsula.frc2024.behavior;

import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.ReadOnly;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.SubsystemBase;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

/** Runs routines one at a time. Finishes when the last one is finished. */
public class SequentialRoutine extends MultipleRoutineBase {

  private final Iterator<RoutineBase> mIterator = mRoutines.iterator();
  private RoutineBase mRunningRoutine = mIterator.next();

  public SequentialRoutine(RoutineBase... routines) {
    super(routines);
  }

  public SequentialRoutine(List<RoutineBase> routines) {
    super(routines);
  }

  @Override
  public void update(Commands commands, @ReadOnly RobotState state) {
    while (mRunningRoutine.execute(commands, state)) {
      if (!mIterator.hasNext()) {
        mRunningRoutine = null;
        break;
      }
      mRunningRoutine = mIterator.next();
      //      Log.info(String.format("Moving onto next routine: %s", mRunningRoutine));
    }
  }

  @Override
  protected void stop(Commands commands, @ReadOnly RobotState state) {
    if (mRunningRoutine != null) mRunningRoutine.stop(commands, state);
  }

  @Override
  public boolean checkFinished(@ReadOnly RobotState state) {
    return mRunningRoutine == null;
  }

  @Override
  public Set<SubsystemBase> getRequiredSubsystems() {
    return RoutineManager.sharedSubsystems(mRoutines);
  }
}
