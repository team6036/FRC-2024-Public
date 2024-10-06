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

public class Millenium implements AutoBase {
  @Override
  public RoutineBase getRoutine() {

    PeninsulaTrajectory p1 = null;
    PeninsulaTrajectory p2 = null;
    PeninsulaTrajectory p3 = null;
    PeninsulaTrajectory p4 = null;
    PeninsulaTrajectory p5 = null;

    try {
      p1 = PeninsulaTrajectory.of("trajectories/Millenium/FirstOut/data.out");
      p2 = PeninsulaTrajectory.of("trajectories/Millenium/SecondOut/data.out");
      p3 = PeninsulaTrajectory.of("trajectories/Millenium/InFirst/data.out");
      p4 = PeninsulaTrajectory.of("trajectories/Millenium/InSecond/data.out");
      p5 = PeninsulaTrajectory.of("trajectories/Millenium/Last/data.out");
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
            new ControlShooterRoutine(Shooter.State.SET_SPEED, 1.1),
            new SequentialRoutine(
                new AimRoutine(0.75), // Aim. Accelerate shooter.
                new ControlKickerRoutine(Shooter.KickerState.KICK, 0.3)), // Shoot Preload
            new SequentialRoutine(
                new TimedRoutine(0.75), // Wait for the aim, begin movement before shot finishes
                new ParallelRoutine(
                    new SequentialRoutine(
                        new TimedRoutine(0.8), // Wait for the first shot to finish
                        new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.3)),
                    new SequentialRoutine(new TimedRoutine(1.2), new IntakeRoutine(1.7)),
                    new DrivePathRoutine(new WantedTrajectory(p1), 1.1)) // Pick up center 1
                )),
        new AimRoutine(0.4),
        new ControlKickerRoutine(Shooter.KickerState.KICK, 0.3), // Shoot center 1
        new ParallelRoutine(
            new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.3),
            new SequentialRoutine(new TimedRoutine(0.7), new IntakeRoutine(1.7)),
            new DrivePathRoutine(new WantedTrajectory(p2), 1.1)),
        new AimRoutine(0.4),
        new ControlKickerRoutine(Shooter.KickerState.KICK, 0.3), // Shoot center 2
        new ParallelRoutine(
            new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.3),
            new SequentialRoutine(new TimedRoutine(0.7), new IntakeRoutine(1.7)),
            new DrivePathRoutine(new WantedTrajectory(p3), 1.1)), // Pick up close 1
        new AimRoutine(0.4),
        new ControlKickerRoutine(Shooter.KickerState.KICK, 0.3), // Shoot close 1
        new ParallelRoutine(
            new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.3),
            new SequentialRoutine(new TimedRoutine(0.2), new IntakeRoutine(2.0)),
            new DrivePathRoutine(new WantedTrajectory(p4), 1.1)), // Pick up Close 2
        new ParallelRoutine(
            new SequentialRoutine(
                new AimRoutine(0.4),
                new ControlKickerRoutine(Shooter.KickerState.KICK, 0.3)), // Shoot close 2
            new SequentialRoutine(
                new TimedRoutine(0.8),
                new ParallelRoutine(
                    new IntakeRoutine(1.7),
                    new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.3))),
            new DrivePathRoutine(new WantedTrajectory(p5), 1.1)), // Pick up Close 3
        new AimRoutine(0.3),
        new ControlKickerRoutine(Shooter.KickerState.KICK, 0.5) // Shoot 3
        );
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
