package com.peninsula.frc2024.behavior.routines.drive;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.peninsula.frc2024.behavior.TimeoutRoutineBase;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.ReadOnly;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.SubsystemBase;
import com.peninsula.frc2024.util.peninsulaCoolios.phronesis.fieldAwareness.FieldState;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.PeninsulaTrajectory;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.WantedTrajectory;
import java.util.Set;

/** Follows a pathplanner trajectory */
public class DrivePathRoutineCoast extends TimeoutRoutineBase {

  private final WantedTrajectory mTrajectory;
  private boolean start = false;

  private Commands state;

  private FieldState.PieceType targeting = FieldState.PieceType.NONE;
  private int targetN = -1;

  public DrivePathRoutineCoast(WantedTrajectory trajectory) {
    this(trajectory, 1.15);
  }

  public DrivePathRoutineCoast(WantedTrajectory trajectory, double mult_constant) {
    mTrajectory = trajectory;
    mTimeout =
        mTrajectory.getType() == WantedTrajectory.Type.PATH_PLANNER
            ? ((PathPlannerTrajectory) mTrajectory.getTrajectory()).getTotalTimeSeconds()
                * mult_constant
            : ((PeninsulaTrajectory) mTrajectory.getTrajectory()).getTotalTimeSeconds()
                * mult_constant;
  }

  public DrivePathRoutineCoast(
      WantedTrajectory trajectory, double mult_constant, FieldState.PieceType type, int number) {
    mTrajectory = trajectory;
    mTimeout =
        mTrajectory.getType() == WantedTrajectory.Type.PATH_PLANNER
            ? ((PathPlannerTrajectory) mTrajectory.getTrajectory()).getTotalTimeSeconds()
                * mult_constant
            : ((PeninsulaTrajectory) mTrajectory.getTrajectory()).getTotalTimeSeconds()
                * mult_constant;

    this.targeting = type;
    this.targetN = number;
  }

  @Override
  public void start(Commands commands, @ReadOnly RobotState state) {
    // Required to start the timeout timer
    super.start(commands, state);
  }

  @Override
  public Set<SubsystemBase> getRequiredSubsystems() {
    return Set.of(mDrive);
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    if (!start) {
      commands.setDriveFollowPath(mTrajectory);
      this.state = commands;
      state.currentTrajectory = mTrajectory;
      start = true;
    }

    if (targeting.equals(FieldState.PieceType.CENTERLINE)) {

      state.m_field.getObject("NOTES/c1").setPose(state.state.center_observations[0].location());
      state.m_field.getObject("NOTES/c2").setPose(state.state.center_observations[1].location());
      state.m_field.getObject("NOTES/c3").setPose(state.state.center_observations[2].location());
      state.m_field.getObject("NOTES/c4").setPose(state.state.center_observations[3].location());
      state.m_field.getObject("NOTES/c5").setPose(state.state.center_observations[4].location());

      commands.yCorrection = state.state.center_observations[targetN - 1].location().getY();
      commands.correctWanted = true;
    } else {
      commands.correctWanted = false;
    }
  }

  @Override
  public boolean checkIfFinishedEarly(@ReadOnly RobotState state) {
    // TODO: possibly implement to see if we are within a tolerance of the end early
    return false;
  }
}
