package com.peninsula.frc2024.subsystems.controllers.drive;

import com.peninsula.frc2024.config.SwerveConstants;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.Swerve;
import com.peninsula.frc2024.util.control.SwerveOutputs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class FieldRelativeAngleController extends Swerve.SwerveController {

  PIDController angleSetPIDController;

  public FieldRelativeAngleController(SwerveOutputs outputs) {
    super(outputs);
    angleSetPIDController =
        new PIDController(
            SwerveConstants.kSetAngleGains.p,
            SwerveConstants.kSetAngleGains.i,
            SwerveConstants.kSetAngleGains.d);
  }

  // Override this for shoot on move controller
  public double getAngle(Commands commands, RobotState state) {
    return commands.angleWanted;
  }

  @Override
  public void updateSignal(Commands commands, RobotState state) {
    double x =
        state.driverLeftX
            * (commands.boostWanted
                ? SwerveConstants.kTeleopBoostMaxTransVel
                : SwerveConstants.kTeleopMaxTransVel);
    double y =
        state.driverLeftY
            * (commands.boostWanted
                ? SwerveConstants.kTeleopBoostMaxTransVel
                : SwerveConstants.kTeleopMaxTransVel);
    double z =
        angleSetPIDController.calculate(
            state.poseEst.getEstimatedPosition().getRotation().getDegrees(),
            getAngle(commands, state));

    SwerveModuleState[] moduleStates =
        SwerveConstants.kKinematics.toSwerveModuleStates(
            commands.robotCentricWanted
                ? new ChassisSpeeds(-x, y, z)
                : ChassisSpeeds.fromFieldRelativeSpeeds(-x, y, z, state.gyroHeading));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        moduleStates, SwerveConstants.Constants.Swerve.maxSpeed);
    mOutputs.setOutputs(moduleStates);
  }
}
