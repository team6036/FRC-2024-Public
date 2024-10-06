package com.peninsula.frc2024.auto.drive;

import com.peninsula.frc2024.auto.AutoBase;
import com.peninsula.frc2024.behavior.ParallelRaceRoutine;
import com.peninsula.frc2024.behavior.RoutineBase;
import com.peninsula.frc2024.behavior.SequentialRoutine;
import com.peninsula.frc2024.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2024.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2024.behavior.routines.statesetters.*;
import com.peninsula.frc2024.robot.Robot;
import com.peninsula.frc2024.util.Flipper;
import com.peninsula.frc2024.util.peninsulaCoolios.phronesis.fieldAwareness.FieldState;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.PeninsulaTrajectory;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.WantedTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import java.io.IOException;

public class CleanSideFourThree implements AutoBase {
  @Override
  public RoutineBase getRoutine() {

    PeninsulaTrajectory p2 = null;
    PeninsulaTrajectory p3 = null;

    try {
      p2 = PeninsulaTrajectory.of("trajectories/CleanSideCenterRush/Center4c/data.out");
      p3 = PeninsulaTrajectory.of("trajectories/CleanSideCenterRush/Center3b/data.out");
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    Pose2d inital = p2.getInitialPose();

    if (Robot.onBlueAlliance) {
      inital = Flipper.flip(inital);
    }

    return new SequentialRoutine(
        new DriveSetOdometryRoutine(
            inital.getX(), inital.getY(), inital.getRotation().getDegrees()),
        new ParallelRaceRoutine(
            new ContinuousIntakeAimRoutine(30),
            new SequentialRoutine(
                // Shoot preload
                new ShootWhenReadyRoutine(0.5, 0.8),
                // Drive center 4
                new DrivePathRoutine(
                    new WantedTrajectory(p2), 1.15, FieldState.PieceType.CENTERLINE, 4),
                // Shoot center 4
                new ShootWhenReadyRoutine(0.4, 0.7, 0.0015),
                // Drive center 3
                new DrivePathRoutine(new WantedTrajectory(p3), 1.1),
                // Shoot center 3
                new ShootWhenReadyRoutine(0.4, 0.8, 0.0015))));
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
