package com.peninsula.frc2024.subsystems.controllers.drive;

import com.peninsula.frc2024.config.SwerveConstants;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.Robot;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.Swerve;
import com.peninsula.frc2024.util.Util;
import com.peninsula.frc2024.util.control.SwerveOutputs;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class LaneKeepController extends Swerve.SwerveController {
  private static HolonomicDriveController mTrajectoryController;
  private static ProfiledPIDController linearController;
  private static ProfiledPIDController thetaController;
  public static boolean resetThetaController =
      false; // Used to reset ProfiledPIDController when driver aligns.

  public LaneKeepController(SwerveOutputs outputs) {
    super(outputs);
    linearController =
        new ProfiledPIDController(
            SwerveConstants.p_xy,
            SwerveConstants.i_xy,
            SwerveConstants.d_xy,
            new TrapezoidProfile.Constraints(SwerveConstants.Constants.Swerve.maxSpeed, 5.0));
    thetaController =
        new ProfiledPIDController(
            SwerveConstants.p_t,
            SwerveConstants.i_t,
            SwerveConstants.d_t,
            new TrapezoidProfile.Constraints(3 * Math.PI, Math.PI * 3));

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    linearController.setTolerance(0.05, 0.02);
    thetaController.setTolerance(0.05, 0.05);
  }

  @Override
  public void updateSignal(Commands commands, RobotState state) {

    double inputRange = Math.pow(state.driverLeftX, 2) + Math.pow(state.driverLeftY, 2);
    Rotation2d inputAngle =
        new Rotation2d(Math.atan2(state.driverLeftY, state.driverLeftX))
            .plus(Robot.onBlueAlliance ? Rotation2d.fromDegrees(-90) : Rotation2d.fromDegrees(90));

    double longitudinal = inputRange * inputAngle.minus(commands.wantedFSDLaneAngle).getCos();

    double lateral =
        linearController.calculate(
            Util.calculatePerpendicularDistance(
                state.lastEst, commands.wantedFSDPose, commands.wantedFSDLaneAngle),
            0);

    double x =
        lateral * commands.wantedFSDLaneAngle.getSin()
            + longitudinal * commands.wantedFSDLaneAngle.getCos();
    double y =
        lateral * commands.wantedFSDLaneAngle.getCos()
            + longitudinal * commands.wantedFSDLaneAngle.getSin();

    if (resetThetaController) {
      thetaController.reset(state.poseEst.getEstimatedPosition().getRotation().getRadians());
      resetThetaController = false;
    }

    double z =
        thetaController.calculate(
            state.poseEst.getEstimatedPosition().getRotation().getRadians(),
            commands.wantedFSDPose.getRotation().getRadians());

    ChassisSpeeds wantedChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            x, y, z, state.poseEst.getEstimatedPosition().getRotation());

    mOutputs =
        TeleopController.outputGenerator.generateSetpoint(
            TeleopController.limits, mOutputs, state, wantedChassisSpeeds, 0.02);
  }
}
