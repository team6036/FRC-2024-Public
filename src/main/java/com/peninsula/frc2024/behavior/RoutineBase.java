package com.peninsula.frc2024.behavior;

import static java.util.Map.entry;

import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.ReadOnly;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.SubsystemBase;
import com.peninsula.frc2024.subsystems.Swerve;
import com.peninsula.frc2024.util.Util;
import java.util.Map;
import java.util.Set;

public abstract class RoutineBase {

  private enum State {
    INIT,
    RUNNING,
    FINISHED
  }

  private static final Map<State, String> kStateToStatus =
      Map.ofEntries(
          entry(State.INIT, "Not Started"),
          entry(State.RUNNING, "Running"),
          entry(State.FINISHED, "Finished"));

  /** Only should be used for {@link #getRequiredSubsystems()} */
  protected final Swerve mDrive = Swerve.getInstance();

  private State mState = State.INIT;

  /**
   * Handles changing the {@link #mState}.
   *
   * @param commands Routines can only modify desired {@link Commands}.
   * @param state Should only read {@link RobotState}.
   * @return Is the routine finished.
   * @see #start(Commands, RobotState)
   * @see #update(Commands, RobotState)
   */
  public final boolean execute(Commands commands, @ReadOnly RobotState state) {
    if (mState == State.INIT) {
      // Check if a routine is finished before even starting it.
      // This avoids calling any sort of update that would modify Commands when not
      // required.
      if (checkFinished(state)) {
        mState = State.FINISHED;
        return true;
      } else {
        start(commands, state);
        mState = State.RUNNING;
      }
    } else if (mState == State.FINISHED) {
      throw new IllegalStateException(
          String.format("Routine %s already finished! Should not be updated.", toString()));
    }
    update(commands, state);
    if (checkFinished(state)) {
      stop(commands, state);
      mState = State.FINISHED;
      return true;
    }
    return false;
  }

  /** Called after the last update cycle. Not called if the routine finishes immediately. */
  protected void stop(Commands commands, @ReadOnly RobotState state) {}

  /** Check with {@link RobotState} to see if we are done and should no longer be updated. */
  public abstract boolean checkFinished(@ReadOnly RobotState state);

  /**
   * Called on the first update cycle before {@link #update(Commands, RobotState)}, unless we are
   * already finished.
   *
   * @see #checkFinished(RobotState)
   */
  protected void start(Commands commands, @ReadOnly RobotState state) {}

  @Override
  public String toString() {
    return getName();
  }

  /**
   * Called every update until we are finished.
   *
   * @see #checkFinished(RobotState)
   */
  protected void update(Commands commands, @ReadOnly RobotState state) {}

  public String getName() {
    return Util.classToJsonName(getClass());
  }

  public String getStatus() {
    return kStateToStatus.get(mState);
  }

  /**
   * @see #checkFinished(RobotState)
   */
  public final boolean isFinished() {
    return mState == State.FINISHED;
  }

  /** Store subsystems which are required by this routine, preventing routines from overlapping */
  public abstract Set<SubsystemBase> getRequiredSubsystems();
}
