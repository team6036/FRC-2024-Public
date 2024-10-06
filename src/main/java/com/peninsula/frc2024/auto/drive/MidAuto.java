package com.peninsula.frc2024.auto.drive;

import com.peninsula.frc2024.auto.AutoBase;
import com.peninsula.frc2024.behavior.ParallelRoutine;
import com.peninsula.frc2024.behavior.RoutineBase;
import com.peninsula.frc2024.behavior.SequentialRoutine;
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

public class MidAuto implements AutoBase {
  @Override
  public RoutineBase getRoutine() {
    PeninsulaTrajectory top = null;
    PeninsulaTrajectory bottom = null;
    PeninsulaTrajectory vers = null;

    try {
      top = PeninsulaTrajectory.of("trajectories/TwoFourThreeCenterRush/Center2/data.out");
      bottom = PeninsulaTrajectory.of("trajectories/TwoFourThreeCenterRush/Center4/data.out");
      vers = PeninsulaTrajectory.of("trajectories/TwoFourThreeCenterRush/Center3/data.out");
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    Pose2d inital = top.getInitialPose();

    if (Robot.onBlueAlliance) {
      inital = Flipper.flip(inital);
    }

    return new SequentialRoutine(
        new DriveSetOdometryRoutine(
            inital.getX(), inital.getY(), inital.getRotation().getDegrees()),
        new ParallelRoutine(
            new ControlShooterRoutine(Shooter.State.SET_SPEED, 0.7), new AimRoutine(0.4)),
        new ParallelRoutine(
            new DrivePathRoutine(new WantedTrajectory(top), 1.1),
            new SequentialRoutine(
                new ControlKickerRoutine(Shooter.KickerState.KICK, 0.4),
                new ParallelRoutine(
                    new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.1),
                    new IntakeRoutine(3.0)))),
        new AimRoutine(0.4),
        new ControlKickerRoutine(Shooter.KickerState.KICK, 0.3),
        new ParallelRoutine(
            new DrivePathRoutine(new WantedTrajectory(bottom), 1.1),
            new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.1),
            new IntakeRoutine(3.0)),
        new AimRoutine(0.4),
        new ControlKickerRoutine(Shooter.KickerState.KICK, 0.3),
        new ParallelRoutine(
            new DrivePathRoutine(new WantedTrajectory(vers), 1.1),
            new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.1),
            new IntakeRoutine(3.0)),
        new AimRoutine(0.4),
        new ControlKickerRoutine(Shooter.KickerState.KICK, 0.3));
  }
}
