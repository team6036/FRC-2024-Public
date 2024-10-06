package com.peninsula.frc2024.auto.drive;

import com.peninsula.frc2024.auto.AutoBase;
import com.peninsula.frc2024.behavior.ParallelRaceRoutine;
import com.peninsula.frc2024.behavior.RoutineBase;
import com.peninsula.frc2024.behavior.SequentialRoutine;
import com.peninsula.frc2024.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2024.behavior.routines.statesetters.ContinuousIntakeAimRoutine;
import com.peninsula.frc2024.behavior.routines.statesetters.ShootWhenReadyRoutine;
import com.peninsula.frc2024.robot.Robot;
import com.peninsula.frc2024.util.Flipper;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.PeninsulaTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import java.io.IOException;

public class SevenPreloadTest implements AutoBase {
  @Override
  public RoutineBase getRoutine() {
    PeninsulaTrajectory p1 = null;
    try {
      p1 = PeninsulaTrajectory.of("trajectories/7_note/path1/data.out");
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
            new ContinuousIntakeAimRoutine(30), new ShootWhenReadyRoutine(0.3, 0.8)));
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
