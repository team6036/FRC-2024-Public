package com.peninsula.frc2024.auto.drive;

import com.peninsula.frc2024.auto.AutoBase;
import com.peninsula.frc2024.behavior.ParallelRoutine;
import com.peninsula.frc2024.behavior.RoutineBase;
import com.peninsula.frc2024.behavior.SequentialRoutine;
import com.peninsula.frc2024.behavior.routines.TimedRoutine;
import com.peninsula.frc2024.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2024.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2024.behavior.routines.statesetters.*;
import com.peninsula.frc2024.robot.Robot;
import com.peninsula.frc2024.subsystems.Shooter;
import com.peninsula.frc2024.subsystems.Vision;
import com.peninsula.frc2024.util.Flipper;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.PeninsulaTrajectory;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.WantedTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import java.io.IOException;

public class SixPieceCloseAuto implements AutoBase {
  @Override
  public RoutineBase getRoutine() {

    PeninsulaTrajectory p1 = null;
    PeninsulaTrajectory p2 = null;
    PeninsulaTrajectory p3 = null;
    PeninsulaTrajectory p4 = null;
    PeninsulaTrajectory p5 = null;

    try {
      p1 = PeninsulaTrajectory.of("trajectories/4Piece/MoveBack/data.out");
      p2 = PeninsulaTrajectory.of("trajectories/4Piece/ToSecondPiece/data.out");
      p3 = PeninsulaTrajectory.of("trajectories/4Piece/ToThirdPiece/data.out");
      p4 = PeninsulaTrajectory.of("trajectories/4Piece/ToCenterAndShoot/data.out");
      p5 = PeninsulaTrajectory.of("trajectories/4Piece/ToCenter2AndShoot/data.out");
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
            new ControlShooterRoutine(Shooter.State.SET_SPEED, 1.5),
            new AimRoutine(1.5),
            new SequentialRoutine(
                new TimedRoutine(1.0), new ControlKickerRoutine(Shooter.KickerState.KICK, 0.5))),
        new ParallelRoutine(
            new SequentialRoutine(
                new TimedRoutine(0.5), new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.1)),
            new SequentialRoutine(new TimedRoutine(0.5), new IntakeRoutine(1.0)),
            new DrivePathRoutine(new WantedTrajectory(p1), 1.5)),
        new AimRoutine(0.5),
        new ParallelRoutine(
            new SequentialRoutine(
                new ControlKickerRoutine(Shooter.KickerState.KICK, 0.5), // Shoot first piece
                new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.1)),
            new SequentialRoutine(new TimedRoutine(0.5), new IntakeRoutine(1.0)),
            new DrivePathRoutine(new WantedTrajectory(p2), 1.1)),
        new AimRoutine(0.5),
        new ParallelRoutine(
            new SequentialRoutine(
                new ControlKickerRoutine(Shooter.KickerState.KICK, 0.5),
                new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.1)),
            new SequentialRoutine(new TimedRoutine(0.3), new IntakeRoutine(1.0)),
            new DrivePathRoutine(new WantedTrajectory(p3), 1.1)),
        new AimRoutine(0.5),
        new ControlKickerRoutine(Shooter.KickerState.KICK, 0.5),
        new ParallelRoutine(
            new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.5),
            new SequentialRoutine(
                new TimedRoutine(1.0),
                new ParallelRoutine(
                    new IntakeRoutine(1.5),
                    new ControlVisionRoutine(Vision.State.OFF, 1.4),
                    new ControlVisionRoutine(Vision.State.ON, 0.1)),
                new AimRoutine(0.5)),
            new DrivePathRoutine(new WantedTrajectory(p4))),
        new ControlKickerRoutine(Shooter.KickerState.KICK, 0.5),
        new ParallelRoutine(
            new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.5),
            new SequentialRoutine(
                new TimedRoutine(0.75),
                new ParallelRoutine(
                    new IntakeRoutine(1.5),
                    new ControlVisionRoutine(Vision.State.OFF, 1.4),
                    new ControlVisionRoutine(Vision.State.ON, 0.1)),
                new AimRoutine(0.5)),
            new DrivePathRoutine(new WantedTrajectory(p5))),
        new ControlKickerRoutine(Shooter.KickerState.KICK, 0.5));
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
