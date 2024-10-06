package com.peninsula.frc2024.subsystems.controllers.drive;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PIDConstants;
import com.peninsula.frc2024.config.SwerveConstants;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.Swerve;
import com.peninsula.frc2024.util.control.SwerveOutputs;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.WantedTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

public class AutoDriveController extends Swerve.SwerveController {

  private final Timer mTimer = new Timer();
  private PeninsulaHolonomicDriveController mTrajectoryController =
      new PeninsulaHolonomicDriveController(
          new PIDController(1.04, 0.0, 0.0),
          new PIDController(1.04, 0.0, 0.0),
          new ProfiledPIDController(
              0.0, 0.00, 0.00, new TrapezoidProfile.Constraints(3 * Math.PI, Math.PI * 3)));

  private PPHolonomicDriveController mTrajPPController =
      new PPHolonomicDriveController(
          new PIDConstants(1.04, 0.0, 0.0),
          new PIDConstants(2.0, 0.0, 0.0),
          SwerveConstants.Constants.Swerve.maxSpeed,
          Math.hypot(SwerveConstants.kOffsetX, SwerveConstants.kOffsetY));

  WantedTrajectory mTrajectory;

  public AutoDriveController(SwerveOutputs outputs) {
    super(outputs);
    mTimer.start();
  }

  private void resetTimer() {
    mTimer.reset();
  }

  @Override
  public void updateSignal(Commands commands, RobotState state) {
    WantedTrajectory wanted = commands.getWantedTrajectory();

    if (wanted.getType() == WantedTrajectory.Type.PATH_PLANNER) {
      if (wanted != mTrajectory) {
        mTrajectory = wanted;
        resetTimer();
        mTrajectoryController = PathPlannerTrajectoryFollower.reset();
      }

      PathPlannerTrajectoryFollower.updateSignal(
          mOutputs, mTrajPPController, state, commands, mTimer.get());
    } else {
      if (wanted != mTrajectory) {
        mTrajectory = wanted;
        resetTimer();
        mTrajectoryController = PeninsulaTrajectoryFollowerWithCorrection.reset();
      }

      PeninsulaTrajectoryFollowerWithCorrection.updateSignal(
          mOutputs, mTrajectoryController, state, commands, mTimer.get());
    }
  }
}
