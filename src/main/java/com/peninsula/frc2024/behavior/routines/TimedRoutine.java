package com.peninsula.frc2024.behavior.routines;

import com.peninsula.frc2024.behavior.RoutineBase;
import com.peninsula.frc2024.behavior.TimeoutRoutineBase;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.ReadOnly;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashSet;
import java.util.Set;

/**
 * Routine that waits the specified amount of time. Does not require any subsystems. The preferred
 * way to extend time based routines is to use {@link TimeoutRoutineBase} instead. Passing {@link
 * Double#POSITIVE_INFINITY} to the timeout can be used to achieve a persistent command.
 */
public class TimedRoutine extends RoutineBase {

  protected final Timer mTimer = new Timer();
  protected double mTimeout;

  /**
   * @see TimedRoutine
   */
  public TimedRoutine(double durationSeconds) {
    mTimeout = durationSeconds;
  }

  @Override
  public void start(Commands commands, @ReadOnly RobotState state) {
    mTimer.start();
  }

  @Override
  public boolean checkFinished(@ReadOnly RobotState state) {
    return mTimer.hasElapsed(mTimeout);
  }

  @Override
  public Set<SubsystemBase> getRequiredSubsystems() {
    return new HashSet<>();
  }
}
