package com.peninsula.frc2024.subsystems.controllers.drive;

import com.peninsula.frc2024.config.SwerveConstants;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.Robot;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.Swerve;
import com.peninsula.frc2024.util.control.SwerveOutputs;
import com.peninsula.frc2024.util.swerveDrivers.SwerveOutputGenerator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TeleopController extends Swerve.SwerveController {
  // TODO, The following lines (limits and outputGenerator) should be put as SwerveConstants.
  public static final SwerveOutputGenerator.KinematicLimits limits =
      new SwerveOutputGenerator.KinematicLimits();

  public static final SwerveOutputGenerator.KinematicLimits slo_limits =
      new SwerveOutputGenerator.KinematicLimits();

  static {
    limits.kMaxDriveVelocity = SwerveConstants.Constants.Swerve.maxSpeed;
    limits.kMaxSteeringVelocity = SwerveConstants.Constants.Swerve.maxAngularVelocity;
    limits.kMaxDriveAcceleration = 10; // Arbitrary

    slo_limits.kMaxDriveVelocity = SwerveConstants.Constants.Swerve.maxSpeed * 0.5;
    slo_limits.kMaxSteeringVelocity = SwerveConstants.Constants.Swerve.maxAngularVelocity;
    slo_limits.kMaxDriveAcceleration = limits.kMaxDriveAcceleration * 0.5; // Arbitrary
  }

  public static final SwerveOutputGenerator outputGenerator =
      new SwerveOutputGenerator(SwerveConstants.kKinematics);

  public TeleopController(SwerveOutputs outputs) {
    super(outputs);
  }

  @Override
  public void updateSignal(Commands commands, RobotState state) {

    double translationSpeed =
        Math.min(
                Math.pow(
                    Math.sqrt(Math.pow(state.driverLeftX, 2) + Math.pow(state.driverLeftY, 2)), 3),
                1)
            * SwerveConstants.kTeleopBoostMaxTransVel;
    Rotation2d translationAngle = new Rotation2d(Math.atan2(state.driverLeftY, state.driverLeftX));
    double x = translationSpeed * translationAngle.getCos();
    double y = translationSpeed * translationAngle.getSin();
    double z =
        -Math.signum(state.driverRightX)
            * state.driverRightX
            * state.driverRightX
            * SwerveConstants.kTeleopBoostMaxRotVel;

    if (Robot.onBlueAlliance) {
      y = -y;
      x = -x;
    }

    ChassisSpeeds wantedChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            y, x, z, state.poseEst.getEstimatedPosition().getRotation());

    // OutputGenerator does not work in sim for some reason
    if (!Robot.isRobotReal()) {
      mOutputs.setOutputs(SwerveConstants.kKinematics.toSwerveModuleStates(wantedChassisSpeeds));
    } else if (commands.boostWanted) {
      mOutputs =
          outputGenerator.generateSetpoint(slo_limits, mOutputs, state, wantedChassisSpeeds, 0.02);
    } else {
      mOutputs =
          outputGenerator.generateSetpoint(limits, mOutputs, state, wantedChassisSpeeds, 0.02);
    }
  }
}
