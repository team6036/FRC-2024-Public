import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

public class SwervePoseTest {

  @Test
  public void testTwistPoseFix() {

    double kOffsetY = Units.inchesToMeters(28.0 / 2);
    double kOffsetX = Units.inchesToMeters(28.0 / 2);

    Translation2d kFLPos = new Translation2d(kOffsetX, kOffsetY);
    Translation2d kFRPos = new Translation2d(kOffsetX, -kOffsetY);
    Translation2d kBLPos = new Translation2d(-kOffsetX, kOffsetY);
    Translation2d kBRPos = new Translation2d(-kOffsetX, -kOffsetY);

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(kFLPos, kFRPos, kBLPos, kBRPos);

    SwerveModulePosition[] modulePositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(0, new Rotation2d(0)),
          new SwerveModulePosition(0, new Rotation2d(0)),
          new SwerveModulePosition(0, new Rotation2d(0)),
          new SwerveModulePosition(0, new Rotation2d(0))
        };

    SwerveDriveOdometry odom =
        new SwerveDriveOdometry(kinematics, new Rotation2d(0), modulePositions);
    odom.update(
        new Rotation2d(0),
        new SwerveModulePosition[] {
          new SwerveModulePosition(0, new Rotation2d()),
          new SwerveModulePosition(0, new Rotation2d()),
          new SwerveModulePosition(0, new Rotation2d()),
          new SwerveModulePosition(0, new Rotation2d())
        });

    double dt = 0.02;

    double vx = 3, vy = 0, omegaRadPerSec = 4;

    double gyro = 0;
    ChassisSpeeds wantedChassisSpeeds;

    for (int i = 0; i < 50; i++) {

      wantedChassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omegaRadPerSec, new Rotation2d(gyro));

      Pose2d poseDeltaWanted =
          new Pose2d(
              wantedChassisSpeeds.vxMetersPerSecond * dt,
              wantedChassisSpeeds.vyMetersPerSecond * dt,
              new Rotation2d(wantedChassisSpeeds.omegaRadiansPerSecond * dt));

      Pose2d startPoseRelative = new Pose2d(0, 0, new Rotation2d(0));

      Twist2d arcTravel = startPoseRelative.log(poseDeltaWanted);

      ChassisSpeeds correctChassisSpeeds =
          new ChassisSpeeds(arcTravel.dx / dt, arcTravel.dy / dt, arcTravel.dtheta / dt);

      SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(correctChassisSpeeds);
      for (int q = 0; q < 4; q++)
        modulePositions[q] =
            new SwerveModulePosition(
                modulePositions[q].distanceMeters + moduleStates[q].speedMetersPerSecond * dt,
                moduleStates[q].angle);

      gyro += wantedChassisSpeeds.omegaRadiansPerSecond * dt;
      odom.update(new Rotation2d(gyro), modulePositions);
    }

    assertEquals(odom.getPoseMeters().getX(), vx, 1e-5);
    assertEquals(odom.getPoseMeters().getY(), vy, 1e-5);
    assertEquals(
        (odom.getPoseMeters().getRotation().getRadians() + Math.PI * 2) % Math.PI,
        omegaRadPerSec % Math.PI,
        1e-5);
  }
}
