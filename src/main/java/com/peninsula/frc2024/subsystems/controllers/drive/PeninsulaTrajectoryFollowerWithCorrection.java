package com.peninsula.frc2024.subsystems.controllers.drive;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.peninsula.frc2024.config.FieldConstants;
import com.peninsula.frc2024.config.SwerveConstants;
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
import java.util.function.Function;

public class PeninsulaTrajectoryFollowerWithCorrection {

  static double transferDistance = 2.5;
  static double transferDistanceBack = 3.5;

  public static Function<RobotState, Double> weightFunctionForTraj =
      (RobotState state) -> {
        double dist =
            Math.abs(FieldConstants.GamePieceLocations.centerlineX - state.lastEst.getX());
        if (dist < transferDistance / 2) {
          return 4 * Math.pow(dist / transferDistance, 3);
        } else if (dist >= transferDistance) {
          return 1.0;
        } else {
          return 1 - Math.pow(-2 * dist / transferDistance + 2, 3) / 2.0;
        }
      };

  public static Function<RobotState, Double> weightFunctionForTrajBack =
      (RobotState state) -> {
        double dist =
            Math.abs(FieldConstants.GamePieceLocations.centerlineX - state.lastEst.getX());
        if (dist < transferDistanceBack / 2) {
          return 4 * Math.pow(dist / transferDistanceBack, 3);
        } else if (dist >= transferDistanceBack) {
          return 1.0;
        } else {
          return 1 - Math.pow(-2 * dist / transferDistanceBack + 2, 3) / 2.0;
        }
      };

  public PeninsulaTrajectoryFollowerWithCorrection(Function<RobotState, Double> weightFunction) {
    weightFunctionForTraj = weightFunction;
  }

  public static PeninsulaHolonomicDriveController reset() {
    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            15, 0.00, 0.5, SwerveConstants.Constants.AutoConstants.kThetaControllerConstraints);
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

      double weightTraj = weightFunctionForTraj.apply(state);
      if (commands.transferBack) {
        weightTraj = weightFunctionForTrajBack.apply(state);
      }

      targetPoseC.heading =
          Flipper.flipRot(new Rotation2d(targetPose.velo_x, targetPose.velo_y).times(1));

      Pose2d ghost = new Pose2d(targetPoseC.positionMeters, targetPoseC.targetHolonomicRotation);
      state.m_field.getObject("ghostPose").setPose(ghost);

      if (commands.correctWanted) {
        targetPoseC.positionMeters =
            new Translation2d(
                targetPoseC.positionMeters.getX(),
                targetPoseC.positionMeters.getY() * weightTraj
                    + (1 - weightTraj) * commands.yCorrection);
      }

      ghost = new Pose2d(targetPoseC.positionMeters, targetPoseC.targetHolonomicRotation);
      state.m_field.getObject("ghostPoseCorrected").setPose(ghost);

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

      double weightTraj = weightFunctionForTraj.apply(state);
      if (commands.transferBack) {
        weightTraj = weightFunctionForTrajBack.apply(state);
      }
      targetPoseC.heading =
          Flipper.flipRot(new Rotation2d(targetPose.velo_x, targetPose.velo_y).times(1));

      Pose2d ghost = new Pose2d(targetPoseC.positionMeters, targetPoseC.targetHolonomicRotation);
      state.m_field.getObject("ghostPose").setPose(ghost);

      if (commands.correctWanted) {
        targetPoseC.positionMeters =
            new Translation2d(
                targetPoseC.positionMeters.getX(),
                targetPoseC.positionMeters.getY() * weightTraj
                    + (1 - weightTraj) * commands.yCorrection);
      }

      ghost = new Pose2d(targetPoseC.positionMeters, targetPoseC.targetHolonomicRotation);
      state.m_field.getObject("ghostPoseCorrected").setPose(ghost);

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
