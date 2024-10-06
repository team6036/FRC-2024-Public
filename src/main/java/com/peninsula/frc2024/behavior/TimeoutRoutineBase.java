package com.peninsula.frc2024.behavior;

import com.peninsula.frc2024.behavior.routines.TimedRoutine;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.ReadOnly;
import com.peninsula.frc2024.robot.RobotState;

/**
 * Preferred base class for routines. Timeout guarantees that the routine will finish even if
 * something goes horribly wrong. The result is that the robot does not lock up with a faulty
 * routine. When overriding {@link #start(Commands, RobotState)}, care must be taken to either call
 * the super method or start the timer. If not, the routine will never have timeout behavior. {@link
 * #onTimeout()} can be overridden to detect when the routine did not finish according to {@link
 * #checkIfFinishedEarly(RobotState)}
 */
public abstract class TimeoutRoutineBase extends TimedRoutine {

  protected TimeoutRoutineBase() {
    this(0.0);
  }

  public TimeoutRoutineBase(double timeoutSeconds) {
    super(timeoutSeconds);
  }

  @Override
  public final boolean checkFinished(@ReadOnly RobotState state) {
    boolean timeoutFinished = super.checkFinished(state);
    if (timeoutFinished) {
      onTimeout();
    }
    return timeoutFinished || checkIfFinishedEarly(state);
  }

  public abstract boolean checkIfFinishedEarly(@ReadOnly RobotState state);

  public void onTimeout() {}
}
