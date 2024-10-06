package com.peninsula.frc2024.subsystems.controllers.drive;

import com.peninsula.frc2024.config.FieldConstants;
import com.peninsula.frc2024.config.SwerveConstants;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.Robot;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.Swerve;
import com.peninsula.frc2024.util.Util;
import com.peninsula.frc2024.util.control.SwerveOutputs;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class FSDController extends Swerve.SwerveController {
  private static ProfiledPIDController xController;
  private static ProfiledPIDController yController;
  private static ProfiledPIDController thetaController;

  public static boolean resetController =
      false; // Used to reset ProfiledPIDController when driver aligns.
  public static boolean ampMode = false;

  public FSDController(SwerveOutputs outputs) {
    super(outputs);

    xController =
        new ProfiledPIDController(
            SwerveConstants.p_xy,
            SwerveConstants.i_xy,
            SwerveConstants.d_xy,
            new TrapezoidProfile.Constraints(SwerveConstants.Constants.Swerve.maxSpeed * 0.7, 5.0));
    yController =
        new ProfiledPIDController(
            SwerveConstants.p_xy,
            SwerveConstants.i_xy,
            SwerveConstants.d_xy,
            new TrapezoidProfile.Constraints(SwerveConstants.Constants.Swerve.maxSpeed * 0.7, 5.0));
    thetaController =
        new ProfiledPIDController(
            SwerveConstants.p_t,
            SwerveConstants.i_t,
            SwerveConstants.d_t,
            new TrapezoidProfile.Constraints(10, 25));

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    xController.setTolerance(0.05, 0.02);
    yController.setTolerance(0.05, 0.02);
    thetaController.setTolerance(0.05, 0.05);
  }

  @Override
  public void updateSignal(Commands commands, RobotState state) {
    if (resetController) {
      xController.setGoal(commands.wantedFSDPose.rotateBy(commands.wantedFSDLaneAngle).getX());
      yController.setGoal(commands.wantedFSDPose.rotateBy(commands.wantedFSDLaneAngle).getY());
      thetaController.setGoal(commands.wantedFSDPose.getRotation().getRadians());
      Translation2d speeds =
          new Translation2d(
                  state.fieldRelativeSpeeds.vxMetersPerSecond,
                  state.fieldRelativeSpeeds.vyMetersPerSecond)
              .rotateBy(commands.wantedFSDLaneAngle);
      xController.reset(state.lastEst.rotateBy(commands.wantedFSDLaneAngle).getX(), speeds.getX());
      yController.reset(state.lastEst.rotateBy(commands.wantedFSDLaneAngle).getY(), speeds.getY());
      thetaController.reset(
          state.lastEst.getRotation().getRadians(),
          state.fieldRelativeSpeeds.omegaRadiansPerSecond);
      resetController = false;
    }
    if (ampMode
        && Util.withinRange(state.lastEst.getX(), commands.wantedFSDPose.getX(), 0.4)
        && Math.abs(
                state
                    .lastEst
                    .getRotation()
                    .minus(commands.wantedFSDPose.getRotation())
                    .getRotations())
            < 0.1) {
      yController.setGoal(FieldConstants.ampFSDTargetRed.getY());
    }
    if (!ampMode
        && state.climbMode
        && Util.withinRange(state.lastEst, commands.wantedFSDPose, 0.2, 0.1)
        && Math.abs(state.chassisRelativeSpeeds.omegaRadiansPerSecond) < 0.1) {
      state.climbUp = true;
      yController.setGoal(commands.secondaryFSDGoal.rotateBy(commands.wantedFSDLaneAngle).getY());
      xController.setGoal(commands.secondaryFSDGoal.rotateBy(commands.wantedFSDLaneAngle).getX());
      thetaController.setGoal(commands.secondaryFSDGoal.getRotation().getRadians());
    }
    double x =
        xController.getSetpoint().velocity
            + xController.calculate(state.lastEst.rotateBy(commands.wantedFSDLaneAngle).getX());

    double y =
        yController.getSetpoint().velocity
            + yController.calculate(state.lastEst.rotateBy(commands.wantedFSDLaneAngle).getY());

    Translation2d wantedXY =
        new Translation2d(x, y).rotateBy(commands.wantedFSDLaneAngle.times(-1));

    double z = thetaController.calculate(state.lastEst.getRotation().getRadians());

    ChassisSpeeds wantedChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            wantedXY.getX(), wantedXY.getY(), z, state.lastEst.getRotation());

    if (!Robot.isRobotReal()) {
      mOutputs.setOutputs(SwerveConstants.kKinematics.toSwerveModuleStates(wantedChassisSpeeds));
    } else {
      mOutputs =
          TeleopController.outputGenerator.generateSetpoint(
              TeleopController.limits, mOutputs, state, wantedChassisSpeeds, 0.02);
    }
  }

  /**
   * Calculates distance from current position and wanted position
   *
   * @param currentPosition the current position of the robot.
   * @param wantedPosition the wanted position of the robot.
   * @return euclidean distance between current position and wanted position.
   */
  public static double getRemainingDistance(Pose2d currentPosition, Pose2d wantedPosition) {
    return currentPosition.minus(wantedPosition).getTranslation().getNorm();
  }

  /**
   * Returns true if target is reached
   *
   * @return
   */
  public static boolean targetReached() {
    //    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    return false;
  }
}
