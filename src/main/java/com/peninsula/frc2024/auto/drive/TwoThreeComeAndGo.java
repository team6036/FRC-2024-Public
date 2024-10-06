package com.peninsula.frc2024.auto.drive;

import com.peninsula.frc2024.auto.AutoBase;
import com.peninsula.frc2024.behavior.ParallelRaceRoutine;
import com.peninsula.frc2024.behavior.ParallelRoutine;
import com.peninsula.frc2024.behavior.RoutineBase;
import com.peninsula.frc2024.behavior.SequentialRoutine;
import com.peninsula.frc2024.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2024.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2024.behavior.routines.statesetters.*;
import com.peninsula.frc2024.robot.Robot;
import com.peninsula.frc2024.util.Flipper;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.PeninsulaTrajectory;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.WantedTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import java.io.IOException;

public class TwoThreeComeAndGo implements AutoBase {
  @Override
  public RoutineBase getRoutine() {

    PeninsulaTrajectory traj1 = null;
    PeninsulaTrajectory traj2 = null;
    PeninsulaTrajectory traj3 = null;

    try {
      traj1 = PeninsulaTrajectory.of("trajectories/TwoThreeComeAndGo/Go2/data.out");
      traj2 = PeninsulaTrajectory.of("trajectories/OneThreeComeAndGo/traj3/data.out");
      traj3 = PeninsulaTrajectory.of("trajectories/tidecomingin/three/data.out");
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    Pose2d inital = traj1.getInitialPose();

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
                new ShootWhenReadyRoutine(1.0, 2.0),
                // Drive back for second center line piece
                new DrivePathRoutine(new WantedTrajectory(traj1), 1.1),
                // Shoot second center line piece
                new ShootWhenReadyRoutine(0.3, 1.3),
                // Drive back for third center line piece
                new DrivePathRoutine(new WantedTrajectory(traj2), 1.1),
                // Shoot third center line piece
                new ShootWhenReadyRoutine(0.3, 1.3))),
        // Sweep close pieces
        new ParallelRoutine(
            new DrivePathRoutine(new WantedTrajectory(traj3), 1.1),
            new ContinuousIntakeShootRoutine(30)));
  }
}
