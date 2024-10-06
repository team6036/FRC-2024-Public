package com.peninsula.frc2024.robot;

import com.peninsula.frc2024.config.SwerveConstants;
import com.peninsula.frc2024.config.TurretConstants;
import com.peninsula.frc2024.config.VisionConstants;
import com.peninsula.frc2024.subsystems.Vision;
import com.peninsula.frc2024.util.Optimize;
import com.peninsula.frc2024.vision.PerceptionObservation;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class that does the necessary calculations for the Robot State(RobotState.java) after the
 * hardware reads performed by the HardwareReader.java class.
 */
public class RobotStateEstimator {
  private static final RobotStateEstimator sInstance = new RobotStateEstimator();

  private RobotStateEstimator() {}

  public static RobotStateEstimator getInstance() {
    return sInstance;
  }

  public void updateSwervePoseEstimator(RobotState state) {

    // +x is shooter side, +y is side with PDP
    state.chassisRelativeSpeeds =
        SwerveConstants.kKinematics.toChassisSpeeds(state.realModuleStates);

    state.fieldRelativeSpeeds =
        new ChassisSpeeds(
            state.lastEst.getRotation().getCos() * state.chassisRelativeSpeeds.vxMetersPerSecond
                - state.lastEst.getRotation().getSin()
                    * state.chassisRelativeSpeeds.vyMetersPerSecond,
            state.lastEst.getRotation().getSin() * state.chassisRelativeSpeeds.vxMetersPerSecond
                + state.lastEst.getRotation().getCos()
                    * state.chassisRelativeSpeeds.vyMetersPerSecond,
            state.chassisRelativeSpeeds.omegaRadiansPerSecond);

    if (Robot.isRobotReal()) {
      state.gyroHeading = state.gyroHeading.plus(state.initPose.getRotation());
    }

    state.poseEst.update(state.gyroHeading, state.realModulePositions);

    // Update vision estimates
    for (PerceptionObservation observation : Vision.getInstance().getVisionOutputs()) {
      state.poseEst.addVisionMeasurement(
          observation.getPose2d(), observation.getTimestampSeconds());
    }

    state.odometryDeadReckonUpdateTime = Timer.getFPGATimestamp();

    state.pastPoses.addSample(
        state.odometryDeadReckonUpdateTime, state.poseEst.getEstimatedPosition());

    solveVirtualGoal(state);
    solveVirtualGoalHoard(state);

    //    if (state.currentTrajectory != null) {
    //
    //      /* Field 2D update */
    //      state.m_field.getObject("traj").setTrajectory(state.currentTrajectory);
    //    }

    SmartDashboard.putData(state.m_field);

    state.m_field.setRobotPose(state.lastEst);

    state.m_field3d.setRobotPose(new Pose3d(state.lastEst));
    state
        .m_field3d
        .getObject("front left")
        .setPose(new Pose3d(state.lastEst).transformBy(VisionConstants.robot_to_camera_front_left));
    state
        .m_field3d
        .getObject("front right")
        .setPose(
            new Pose3d(state.lastEst).transformBy(VisionConstants.robot_to_camera_front_right));
    state
        .m_field3d
        .getObject("back left")
        .setPose(new Pose3d(state.lastEst).transformBy(VisionConstants.robot_to_camera_back_left));
    state
        .m_field3d
        .getObject("back right")
        .setPose(new Pose3d(state.lastEst).transformBy(VisionConstants.robot_to_camera_back_right));

    SmartDashboard.putData(state.m_field3d);

    ct++;
  }

  public void updateSuperstructureViz(RobotState state) {
    Pose3d turret_component =
        new Pose3d(
            0,
            0,
            0,
            new Rotation3d(
                0,
                0,
                Units.rotationsToRadians(state.turretMotorPosition / TurretConstants.gearRatio)));

    state.m_field3d.getObject("turret_component").setPose(turret_component);
  }

  public void solveVirtualGoal(RobotState state) {

    double gx =
        (Robot.onBlueAlliance)
            ? VisionConstants.goal_position_blue.getX()
            : VisionConstants.goal_position_red.getX();
    double gy = VisionConstants.goal_position_red.getY();

    double tof =
        Optimize.getTOF(
            state.poseEst.getEstimatedPosition().getX(),
            state.poseEst.getEstimatedPosition().getY(),
            state.fieldRelativeSpeeds.vxMetersPerSecond,
            state.fieldRelativeSpeeds.vyMetersPerSecond,
            gx,
            gy);

    state.virtual_goal =
        new Pose2d(gx, gy, new Rotation2d())
            .transformBy(
                new Transform2d(
                    new Translation2d(
                        -state.fieldRelativeSpeeds.vxMetersPerSecond * tof,
                        -state.fieldRelativeSpeeds.vyMetersPerSecond * tof),
                    new Rotation2d(0)));

    state.dist_to_target = state.virtual_goal.minus(state.lastEst).getTranslation().getNorm();

    state.m_field.getObject("virtual goal").setPoses(state.virtual_goal);
  }

  public void solveVirtualGoalHoard(RobotState state) {

    double gx =
        (Robot.onBlueAlliance)
            ? VisionConstants.hoard_position_blue.getX()
            : VisionConstants.hoard_position_red.getX();
    double gy = VisionConstants.hoard_position_red.getY();
    //
    //    double tof = 2;
    //
    //    state.hoard_virtual_goal =
    //        new Pose2d(gx, gy, new Rotation2d())
    //            .transformBy(
    //                new Transform2d(
    //                    new Translation2d(
    //                        -state.fieldRelativeSpeeds.vxMetersPerSecond * tof,
    //                        -state.fieldRelativeSpeeds.vyMetersPerSecond * tof),
    //                    new Rotation2d(0)));
    state.hoard_virtual_goal = new Pose2d(gx, gy, new Rotation2d());

    state.m_field.getObject("hoard virtual goal").setPoses(state.hoard_virtual_goal);
  }

  public static int ct = 0;
}
