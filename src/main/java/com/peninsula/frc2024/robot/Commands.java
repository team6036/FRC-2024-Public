package com.peninsula.frc2024.robot;

import com.peninsula.frc2024.behavior.RoutineBase;
import com.peninsula.frc2024.subsystems.*;
import com.peninsula.frc2024.util.peninsulaCoolios.phronesis.fieldAwareness.FieldState;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.WantedTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

/** Commands represent what we want the robot to be doing. */
@SuppressWarnings("java:S1104")
public class Commands {
  private double shooterWantedRPM;

  /* Routines */
  public List<RoutineBase> routinesWanted = new ArrayList<>();
  public boolean shouldClearCurrentRoutines;
  /* Swerve */
  public Swerve.State swerveWanted;
  public boolean boostWanted;
  public boolean robotCentricWanted;
  public double angleWanted;

  /* Auto */
  public WantedTrajectory wantedTrajectory;
  public Pose2d driveWantedOdometryPose = new Pose2d(0, 0, new Rotation2d(0));
  public Rotation2d driveWantedOdometryPoseRotation = new Rotation2d(0);

  /* Vision */
  public Vision.State visionWanted = Vision.State.ON;

  public void addWantedRoutines(RoutineBase... wantedRoutines) {
    for (RoutineBase wantedRoutine : wantedRoutines) {
      addWantedRoutine(wantedRoutine);
    }
  }

  public void addWantedRoutine(RoutineBase wantedRoutine) {
    routinesWanted.add(wantedRoutine);
  }

  /* Drive */
  public Pose2d wantedFSDPose = new Pose2d(new Translation2d(), new Rotation2d());
  public Pose2d secondaryFSDGoal = new Pose2d(new Translation2d(), new Rotation2d());
  public Rotation2d wantedFSDLaneAngle = new Rotation2d();

  @Override
  public String toString() {
    var log = new StringBuilder();
    log.append("Wanted routines: ");
    for (RoutineBase routine : routinesWanted) {
      log.append(routine).append(" ");
    }
    return log.append("\n").toString();
  }

  public WantedTrajectory getWantedTrajectory() {
    return wantedTrajectory;
  }

  public void setDriveFollowPath(WantedTrajectory trajectory) {
    wantedTrajectory = trajectory;
    swerveWanted = Swerve.State.AUTO;
  }

  public void setDriveIdle() {
    swerveWanted = Swerve.State.NEUTRAL;
  }

  public boolean correctWanted = false;
  public boolean transferBack = false;
  public double yCorrection = 8;

  /* Lighting */
  public Lighting.State wantedLighting = Lighting.State.IDLE;

  public Intake.State intakeWanted = Intake.State.OFF;

  public Shooter.State shooterWanted = Shooter.State.OFF;

  public Shooter.BlowerState blowerWanted = Shooter.BlowerState.OFF;

  public Shooter.State getShooterWanted() {
    return shooterWanted;
  }

  public void setShooterRPM(double wantedRPM) {
    shooterWantedRPM = wantedRPM;
  }

  public double getShooterWantedRPM() {
    return shooterWantedRPM;
  }

  /* Turret */
  public Turret.State turretWanted = Turret.State.INTAKE;
  public double ampTurretPosition = 0;

  /* Arm */
  public Arm.State wantedArm = Arm.State.INTAKE;
  public double commandedAngle = 0;
  public Rotation2d commandedArmAngle_small = new Rotation2d();

  /* Shooter */
  public Shooter.KickerState kickerWanted = Shooter.KickerState.HOLD;

  /* Vision */
  public boolean visionOff = false;

  public FieldState.PieceType targetingPieceType = FieldState.PieceType.NONE;
  public int targetingPieceNumber = -1;

  public boolean limelightOff = false;
}
