package com.peninsula.frc2024.auto.drive;

import com.peninsula.frc2024.auto.AutoBase;
import com.peninsula.frc2024.behavior.ParallelRoutine;
import com.peninsula.frc2024.behavior.RoutineBase;
import com.peninsula.frc2024.behavior.SequentialRoutine;
import com.peninsula.frc2024.behavior.routines.TimedRoutine;
import com.peninsula.frc2024.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2024.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2024.behavior.routines.statesetters.ContinuousIntakeShootRoutine;
import com.peninsula.frc2024.robot.Robot;
import com.peninsula.frc2024.util.Flipper;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.PeninsulaTrajectory;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.WantedTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import java.io.IOException;

public class Initial3PieceAuto implements AutoBase {
  @Override
  public RoutineBase getRoutine() {
    PeninsulaTrajectory p1 = null;

    try {
      p1 = PeninsulaTrajectory.of("trajectories/Initial3Piece/wing/data.out");
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    Pose2d inital = p1.getInitialPose();

    if (Robot.onBlueAlliance) {
      inital = Flipper.flip(inital);
    }

    return new SequentialRoutine(
        new DriveSetOdometryRoutine(
            inital.getX(), inital.getY(), inital.getRotation().getDegrees()),
        new ParallelRoutine(
            new SequentialRoutine(
                new TimedRoutine(0.5), new DrivePathRoutine(new WantedTrajectory(p1), 1.1)),
            new ContinuousIntakeShootRoutine(10)));
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
