package com.peninsula.frc2024.subsystems.controllers.drive;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.peninsula.frc2024.config.SwerveConstants;
import com.peninsula.frc2024.logging.Logger;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.Robot;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.util.Flipper;
import com.peninsula.frc2024.util.control.SwerveOutputs;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.PeninsulaTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class PeninsulaTrajectoryFollower {
  public static PeninsulaHolonomicDriveController reset() {
    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            15, 0.00, 0.5, SwerveConstants.Constants.AutoConstants.kThetaControllerConstraints);
    //            15, 0.00, 1.0,
    // SwerveConstants.Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return new PeninsulaHolonomicDriveController(
        new PIDController(SwerveConstants.Constants.AutoConstants.kPXController, 0.0, 0.6),
        new PIDController(SwerveConstants.Constants.AutoConstants.kPYController, 0.0, 0.6),
        thetaController);
  }

  public static void updateSignal(
      SwerveOutputs outputs,
      PeninsulaHolonomicDriveController controller,
      RobotState state,
      Commands commands,
      double time) {
    PeninsulaTrajectory trajectory =
        (PeninsulaTrajectory) commands.getWantedTrajectory().getTrajectory();

    PeninsulaTrajectory.TrajectoryState targetPose = trajectory.sample(time);

    state.m_field.getObject("fed pose").setPose(state.lastEst);

    if (Robot.onBlueAlliance) {
      // On blue, create a copy and flip
      PathPlannerTrajectory.State targetPoseC = new PathPlannerTrajectory.State();

      targetPoseC.positionMeters = Flipper.flipTrans(targetPose.poseMeters.getTranslation());
      targetPoseC.targetHolonomicRotation = Flipper.flipRot(targetPose.poseMeters.getRotation());
      targetPoseC.velocityMps =
          Math.sqrt(targetPose.velo_x * targetPose.velo_x + targetPose.velo_y * targetPose.velo_y);
      targetPoseC.positionMeters =
          new Translation2d(targetPoseC.positionMeters.getX(), targetPoseC.positionMeters.getY());
      targetPoseC.heading =
          Flipper.flipRot(new Rotation2d(targetPose.velo_x, targetPose.velo_y).times(1));

      Pose2d ghost = new Pose2d(targetPoseC.positionMeters, targetPoseC.targetHolonomicRotation);

      state.m_field.getObject("ghostPose").setPose(ghost);
      Logger.getInstance().log("Field/ghostPose", ghost);

      ChassisSpeeds wantedChassisSpeeds =
          controller.calculate(state.lastEst, targetPoseC, targetPoseC.targetHolonomicRotation);

      var speds = SwerveConstants.kKinematics.toSwerveModuleStates(wantedChassisSpeeds);

      SwerveDriveKinematics.desaturateWheelSpeeds(speds, 4.5);

      outputs.setOutputs(
          speds,
          state
              .lastEst
              .getRotation()
              .plus(Rotation2d.fromRadians(wantedChassisSpeeds.omegaRadiansPerSecond).times(0.02)));
    } else {

      PathPlannerTrajectory.State targetPoseC = new PathPlannerTrajectory.State();

      targetPoseC.positionMeters = targetPose.poseMeters.getTranslation().times(1);
      targetPoseC.targetHolonomicRotation = targetPose.poseMeters.getRotation().times(1);
      targetPoseC.velocityMps =
          Math.sqrt(targetPose.velo_x * targetPose.velo_x + targetPose.velo_y * targetPose.velo_y);
      targetPoseC.positionMeters =
          new Translation2d(targetPoseC.positionMeters.getX(), targetPoseC.positionMeters.getY());
      targetPoseC.heading = new Rotation2d(targetPose.velo_x, targetPose.velo_y).times(1);

      Pose2d ghost = new Pose2d(targetPoseC.positionMeters, targetPoseC.targetHolonomicRotation);

      state.m_field.getObject("ghostPose").setPose(ghost);
      Logger.getInstance().log("Field/ghostPose", ghost);

      ChassisSpeeds wantedChassisSpeeds =
          controller.calculate(state.lastEst, targetPoseC, targetPoseC.targetHolonomicRotation);

      var speds = SwerveConstants.kKinematics.toSwerveModuleStates(wantedChassisSpeeds);

      SwerveDriveKinematics.desaturateWheelSpeeds(speds, 4.5);

      outputs.setOutputs(
          speds,
          state
              .lastEst
              .getRotation()
              .plus(Rotation2d.fromRadians(wantedChassisSpeeds.omegaRadiansPerSecond).times(0.02)));
    }
  }
}
