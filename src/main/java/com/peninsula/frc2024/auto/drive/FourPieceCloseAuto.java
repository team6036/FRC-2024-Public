package com.peninsula.frc2024.auto.drive;

import com.peninsula.frc2024.auto.AutoBase;
import com.peninsula.frc2024.behavior.ParallelRoutine;
import com.peninsula.frc2024.behavior.RoutineBase;
import com.peninsula.frc2024.behavior.SequentialRoutine;
import com.peninsula.frc2024.behavior.routines.TimedRoutine;
import com.peninsula.frc2024.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2024.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2024.behavior.routines.statesetters.AimRoutine;
import com.peninsula.frc2024.behavior.routines.statesetters.ControlKickerRoutine;
import com.peninsula.frc2024.behavior.routines.statesetters.ControlShooterRoutine;
import com.peninsula.frc2024.behavior.routines.statesetters.IntakeRoutine;
import com.peninsula.frc2024.robot.Robot;
import com.peninsula.frc2024.subsystems.Shooter;
import com.peninsula.frc2024.util.Flipper;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.PeninsulaTrajectory;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.WantedTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import java.io.IOException;

public class FourPieceCloseAuto implements AutoBase {
  @Override
  public RoutineBase getRoutine() {

    PeninsulaTrajectory p1 = null;
    PeninsulaTrajectory p2 = null;
    PeninsulaTrajectory p3 = null;

    try {
      p1 = PeninsulaTrajectory.of("trajectories/4Piece/MoveBack/data.out");
      p2 = PeninsulaTrajectory.of("trajectories/4Piece/ToSecondPiece/data.out");
      p3 = PeninsulaTrajectory.of("trajectories/4Piece/ToThirdPiece/data.out");
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
            new ControlShooterRoutine(Shooter.State.SLOW, 1.5), new AimRoutine(1.5)),
        new ControlKickerRoutine(Shooter.KickerState.KICK, 0.5),
        new ParallelRoutine(
            new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.1),
            new SequentialRoutine(new TimedRoutine(0.5), new IntakeRoutine(1.0)),
            new SequentialRoutine(
                new TimedRoutine(0.2), new ControlShooterRoutine(Shooter.State.SET_SPEED, 0.5)),
            new DrivePathRoutine(new WantedTrajectory(p1))),
        new AimRoutine(0.5),
        new ControlKickerRoutine(Shooter.KickerState.KICK, 1.0),
        new ParallelRoutine(
            new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.1),
            new DrivePathRoutine(new WantedTrajectory(p2)),
            new IntakeRoutine(1.75)),
        new AimRoutine(0.5),
        new ControlKickerRoutine(Shooter.KickerState.KICK, 1.0),
        new ParallelRoutine(
            new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.1),
            new DrivePathRoutine(new WantedTrajectory(p3)),
            new IntakeRoutine(1.0)),
        new AimRoutine(0.5),
        new ControlKickerRoutine(Shooter.KickerState.KICK, 1.0));
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
