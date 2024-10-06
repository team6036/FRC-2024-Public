package com.peninsula.frc2024.subsystems.controllers.drive;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.peninsula.frc2024.config.SwerveConstants;
import com.peninsula.frc2024.logging.Logger;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.Robot;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.util.Flipper;
import com.peninsula.frc2024.util.control.SwerveOutputs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PathPlannerTrajectoryFollower {
  public static PeninsulaHolonomicDriveController reset() {
    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            SwerveConstants.Constants.AutoConstants.kPThetaController,
            0.00,
            0.00,
            SwerveConstants.Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return new PeninsulaHolonomicDriveController(
        new PIDController(SwerveConstants.Constants.AutoConstants.kPXController, 0.0, 0.1),
        new PIDController(SwerveConstants.Constants.AutoConstants.kPYController, 0.0, 0.1),
        thetaController);
  }

  public static void updateSignal(
      SwerveOutputs outputs,
      PPHolonomicDriveController controller,
      RobotState state,
      Commands commands,
      double time) {
    PathPlannerTrajectory trajectory =
        (PathPlannerTrajectory) commands.getWantedTrajectory().getTrajectory();

    PathPlannerTrajectory.State targetPose = trajectory.sample(time);

    if (Robot.onBlueAlliance) {
      // On blue, create a copy and flip
      PathPlannerTrajectory.State targetPoseC = new PathPlannerTrajectory.State();
      targetPoseC.positionMeters = Flipper.flipTrans(targetPose.positionMeters);
      targetPoseC.targetHolonomicRotation = Flipper.flipRot(targetPose.targetHolonomicRotation);
      targetPoseC.velocityMps = targetPose.velocityMps;
      targetPoseC.heading = Flipper.flipRot(targetPose.heading);
      targetPoseC.constraints = targetPose.constraints;

      Pose2d ghost = new Pose2d(targetPoseC.positionMeters, targetPoseC.targetHolonomicRotation);

      state.m_field.getObject("ghostPose").setPose(ghost);
      Logger.getInstance().log("Field/ghostPose", ghost);

      ChassisSpeeds wantedChassisSpeeds =
          controller.calculateRobotRelativeSpeeds(state.lastEst, targetPoseC);

      outputs.setOutputs(
          SwerveConstants.kKinematics.toSwerveModuleStates(wantedChassisSpeeds),
          targetPoseC.targetHolonomicRotation);
    } else {
      Pose2d ghost = new Pose2d(targetPose.positionMeters, targetPose.targetHolonomicRotation);

      state.m_field.getObject("ghostPose").setPose(ghost);
      Logger.getInstance().log("Field/ghostPose", ghost);

      ChassisSpeeds wantedChassisSpeeds =
          controller.calculateRobotRelativeSpeeds(state.lastEst, targetPose);

      outputs.setOutputs(
          SwerveConstants.kKinematics.toSwerveModuleStates(wantedChassisSpeeds),
          targetPose.targetHolonomicRotation);
    }
  }
}
