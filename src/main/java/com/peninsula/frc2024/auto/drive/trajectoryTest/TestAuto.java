package com.peninsula.frc2024.auto.drive.trajectoryTest;

import com.peninsula.frc2024.auto.AutoBase;
import com.peninsula.frc2024.behavior.FunctionChooseRoutine;
import com.peninsula.frc2024.behavior.RoutineBase;
import com.peninsula.frc2024.behavior.SequentialRoutine;
import com.peninsula.frc2024.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2024.behavior.routines.drive.DrivePathRoutineCoast;
import com.peninsula.frc2024.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2024.robot.Robot;
import com.peninsula.frc2024.util.Flipper;
import com.peninsula.frc2024.util.peninsulaCoolios.phronesis.fieldAwareness.FieldState;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.PeninsulaTrajectory;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.WantedTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;

public class TestAuto implements AutoBase {
  @Override
  public RoutineBase getRoutine() {

    PeninsulaTrajectory p1 = null;
    PeninsulaTrajectory p2 = null;
    PeninsulaTrajectory p3 = null;
    PeninsulaTrajectory p4 = null;

    try {
      p1 = PeninsulaTrajectory.of("trajectories/SmartAuto/Out1/data.out");
      p2 = PeninsulaTrajectory.of("trajectories/SmartAuto/DivertFirst/data.out");
      p3 = PeninsulaTrajectory.of("trajectories/SmartAuto/DivertSecond/data.out");
      p4 = PeninsulaTrajectory.of("trajectories/SmartAuto/DivertThird/data.out");
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    PeninsulaTrajectory finalP1 = p2;
    PeninsulaTrajectory finalP2 = p3;
    PeninsulaTrajectory finalP3 = p4;

    RoutineBase chooserFirst =
        new FunctionChooseRoutine(
            (state -> {
              if (state.state.seeingNote(
                  FieldState.PieceType.CENTERLINE,
                  Robot.onBlueAlliance ? DriverStation.Alliance.Blue : DriverStation.Alliance.Red,
                  1)) {
                return new DrivePathRoutineCoast(
                    new WantedTrajectory(finalP1), 1, FieldState.PieceType.CENTERLINE, 1);
              } else if (state.state.seeingNote(
                  FieldState.PieceType.CENTERLINE,
                  Robot.onBlueAlliance ? DriverStation.Alliance.Blue : DriverStation.Alliance.Red,
                  2)) {
                return new DrivePathRoutineCoast(
                    new WantedTrajectory(finalP2), 1, FieldState.PieceType.CENTERLINE, 2);
              } else
                return new DrivePathRoutineCoast(
                    new WantedTrajectory(finalP3), 1, FieldState.PieceType.CENTERLINE, 3);
            }));

    Pose2d inital = p1.getInitialPose();

    if (Robot.onBlueAlliance) {
      inital = Flipper.flip(inital);
    }

    return new SequentialRoutine(
        new DriveSetOdometryRoutine(
            inital.getX(), inital.getY(), inital.getRotation().getDegrees()),
        new DrivePathRoutine(new WantedTrajectory(p1), 0.98),
        chooserFirst);
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
