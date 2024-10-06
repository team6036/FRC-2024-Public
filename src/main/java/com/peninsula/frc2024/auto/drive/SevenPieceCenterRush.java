package com.peninsula.frc2024.auto.drive;

import com.peninsula.frc2024.auto.AutoBase;
import com.peninsula.frc2024.behavior.ParallelRaceRoutine;
import com.peninsula.frc2024.behavior.RoutineBase;
import com.peninsula.frc2024.behavior.SequentialRoutine;
import com.peninsula.frc2024.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2024.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2024.behavior.routines.statesetters.ContinuousIntakeAimRoutine;
import com.peninsula.frc2024.behavior.routines.statesetters.ShootWhenReadyRoutine;
import com.peninsula.frc2024.robot.Robot;
import com.peninsula.frc2024.util.Flipper;
import com.peninsula.frc2024.util.peninsulaCoolios.phronesis.fieldAwareness.FieldState;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.PeninsulaTrajectory;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.WantedTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import java.io.IOException;

public class SevenPieceCenterRush implements AutoBase {
  @Override
  public RoutineBase getRoutine() {
    PeninsulaTrajectory p1 = null;
    PeninsulaTrajectory p2 = null;
    PeninsulaTrajectory p3 = null;
    PeninsulaTrajectory sweep = null;
    try {
      p1 = PeninsulaTrajectory.of("trajectories/7_note/path1/data.out");
      p2 = PeninsulaTrajectory.of("trajectories/7_note/path2/data.out");
      p3 = PeninsulaTrajectory.of("trajectories/7_note/path3d/data.out");
    } catch (IOException e) {
      throw new RuntimeException(e); // L
    }

    Pose2d inital = p1.getInitialPose();

    if (Robot.onBlueAlliance) {
      inital = Flipper.flip(inital);
    }

    return new SequentialRoutine(
        new DriveSetOdometryRoutine(
            inital.getX(), inital.getY(), inital.getRotation().getDegrees()),
        new ParallelRaceRoutine(
            new ContinuousIntakeAimRoutine(30),
            new SequentialRoutine(
                new ShootWhenReadyRoutine(0.4, 0.8), // TODO: Could be faster
                new DrivePathRoutine(new WantedTrajectory(p1), 1.15),
                new ShootWhenReadyRoutine(0.2, 0.6),
                new DrivePathRoutine(
                    new WantedTrajectory(p2), 1.15, FieldState.PieceType.CENTERLINE, 2, 0.1),
                new ShootWhenReadyRoutine(0.2, 0.6),
                new DrivePathRoutine(new WantedTrajectory(p3), 1.05),
                new ShootWhenReadyRoutine(0.5, 1.5))));
    //        new ParallelRoutine(
    //            new ContinuousIntakeShootRoutine(30),
    //            new DrivePathRoutine(new WantedTrajectory(sweep), 1.1)));
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
