package com.peninsula.frc2024.auto.drive;

import com.peninsula.frc2024.auto.AutoBase;
import com.peninsula.frc2024.behavior.ParallelRaceRoutine;
import com.peninsula.frc2024.behavior.RoutineBase;
import com.peninsula.frc2024.behavior.SequentialRoutine;
import com.peninsula.frc2024.behavior.routines.TimedRoutine;
import com.peninsula.frc2024.behavior.routines.statesetters.ContinuousIntakeAimRoutine;
import com.peninsula.frc2024.behavior.routines.statesetters.ShootWhenReadyRoutine;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.PeninsulaTrajectory;
import java.io.IOException;

public class Preload implements AutoBase {
  @Override
  public RoutineBase getRoutine() {

    PeninsulaTrajectory go = null;
    try {
      go = PeninsulaTrajectory.of("trajectories/tidecomingin/go1/data.out");
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    //    Pose2d inital = new Pose2d(new Translation2d(15.190617, 7.357184),
    // Rotation2d.fromRadians(0));
    //
    //    if (Robot.onBlueAlliance) {
    //      inital = Flipper.flip(inital);
    //    }

    return new SequentialRoutine(
        //        new DriveSetOdometryRoutine(
        //            inital.getX(), inital.getY(), inital.getRotation().getDegrees()),
        new ParallelRaceRoutine(
            new ContinuousIntakeAimRoutine(30),
            new SequentialRoutine(
                new TimedRoutine(5),
                // Shoot preload
                new ShootWhenReadyRoutine(1.0, 2.0))));
    //                new TimedRoutine(5),
    //                new DrivePathRoutine(new WantedTrajectory(go), 1.1))));
  }
}
