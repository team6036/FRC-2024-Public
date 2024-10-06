package com.peninsula.frc2024.behavior;

import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.ReadOnly;
import com.peninsula.frc2024.robot.Robot;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.SubsystemBase;
import com.peninsula.frc2024.util.Util;
import java.util.*;

/**
 * Handles the updating of commands by passing them to each running routine.
 *
 * @author Quintin
 */
public class RoutineManager {

  public static final String kLoggerTag = Util.classToJsonName(RoutineManager.class);
  private List<RoutineBase> mRunningRoutines = new LinkedList<>();

  static Set<SubsystemBase> sharedSubsystems(List<RoutineBase> routines) {
    Set<SubsystemBase> sharedSubsystems = new HashSet<>(); // TODO: No allocation on update
    for (RoutineBase routine : routines) {
      sharedSubsystems.addAll(routine.getRequiredSubsystems());
    }
    return sharedSubsystems;
  }

  public List<RoutineBase> getCurrentRoutines() {
    return mRunningRoutines;
  }

  /**
   * Updates the commands that are passed in based on the running routines. Removes routines that
   * have finished. Adds wanted routines. Drops current running ones that have conflicting
   * subsystems.
   */
  public void update(Commands commands, @ReadOnly RobotState state) {
    mRunningRoutines.removeIf(
        routine -> {
          boolean isFinished = routine.execute(commands, state);
          if (isFinished) {
            //            Log.debug(kLoggerTag, String.format("Dropping finished routine: %s%n",
            // routine));
          }
          return isFinished;
        });
    if (commands.shouldClearCurrentRoutines) {
      clearRunningRoutines();
    }
    for (RoutineBase newRoutine : commands.routinesWanted) {
      // Remove any running routines that conflict with new routine
      for (RoutineBase runningRoutine : mRunningRoutines) {
        // Non-disjoint means both subsystem sets have elements in common, which creates
        // conflicts
        if (!Collections.disjoint(
            newRoutine.getRequiredSubsystems(), runningRoutine.getRequiredSubsystems())) {
          //          Log.warn(kLoggerTag, String.format("Dropping conflicting routine: %s%n",
          // runningRoutine));
          mRunningRoutines.remove(runningRoutine);
          runningRoutine.stop(commands, state);
        }
      }
      // If it finishes immediately never add it to running routines
      if (newRoutine.execute(commands, state)) {
        //        Log.debug(kLoggerTag, String.format("Immediately dropping new routine: %s%n",
        // newRoutine));
      } else {
        //        Log.debug(kLoggerTag, String.format("Adding routine: %s%n", newRoutine));
        mRunningRoutines.add(newRoutine);
      }
    }
    // Clears the wanted routines every update cycle
    commands.routinesWanted.clear();
    Robot.sLoopDebugger.addPoint("routineManagerUpdate");
  }

  public void clearRunningRoutines() {
    mRunningRoutines.clear();
  }
}
