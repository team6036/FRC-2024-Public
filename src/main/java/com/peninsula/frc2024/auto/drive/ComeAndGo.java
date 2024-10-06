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
import com.peninsula.frc2024.util.peninsulaCoolios.phronesis.fieldAwareness.FieldState;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.PeninsulaTrajectory;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.WantedTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import java.io.IOException;

public class ComeAndGo implements AutoBase {
  @Override
  public RoutineBase getRoutine() {

    PeninsulaTrajectory go = null;
    PeninsulaTrajectory come = null;

    try {
      go = PeninsulaTrajectory.of("trajectories/tidecominginautos/first/data.out");
      come = PeninsulaTrajectory.of("trajectories/tidecominginautos/second/data.out");
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    Pose2d inital = go.getInitialPose();

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
                new ShootWhenReadyRoutine(0.4, 0.8),
                // Drive back for first center line piece
                new DrivePathRoutine(
                    new WantedTrajectory(go), 1.25, FieldState.PieceType.CENTERLINE, 1),
                // Shoot first center line piece
                new ShootWhenReadyRoutine(0.5, 1.3)
                // Drive back for second center line piece
                )
            // Shoot second center line piece
            ),
        new ParallelRoutine(
            new SequentialRoutine(
                new ContinuousIntakeAimRoutine(2.5), new ContinuousIntakeShootRoutine(30)),
            new DrivePathRoutine(
                new WantedTrajectory(come), 1.15, FieldState.PieceType.CENTERLINE, 2)));
  }
}
