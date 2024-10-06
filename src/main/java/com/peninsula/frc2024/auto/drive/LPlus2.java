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
import com.peninsula.frc2024.util.Flipper;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.PeninsulaTrajectory;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.WantedTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import java.io.IOException;

public class LPlus2 implements AutoBase {
  @Override
  public RoutineBase getRoutine() {

    PeninsulaTrajectory p1 = null;
    PeninsulaTrajectory p2 = null;
    PeninsulaTrajectory p3 = null;
    PeninsulaTrajectory p4 = null;
    PeninsulaTrajectory p5 = null;
    PeninsulaTrajectory p6 = null;

    try {
      p1 = PeninsulaTrajectory.of("trajectories/L_plus_2/Close3/data.out");
      p2 = PeninsulaTrajectory.of("trajectories/L_plus_2/Close2/data.out");
      p3 = PeninsulaTrajectory.of("trajectories/L_plus_2/Close1/data.out");
      p4 = PeninsulaTrajectory.of("trajectories/L_plus_2/Center1/data.out");
      p5 = PeninsulaTrajectory.of("trajectories/L_plus_2/Center2/data.out");
      p6 = PeninsulaTrajectory.of("trajectories/L_plus_2/Center3/data.out");
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
                new AimRoutine(1), // Aim. Accelerate shooter.
                new ParallelRoutine(
                    new SequentialRoutine(
                        new ControlKickerRoutine(Shooter.KickerState.KICK, 0.3), // Shoot Preload
                        new ParallelRoutine(
                            new IntakeRoutine(1.7),
                            new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.3))),
                    new SequentialRoutine(
                        new TimedRoutine(0.1), // Delay the drive/wait for kick
                        new DrivePathRoutine(new WantedTrajectory(p1), 1.1))
                    // Drive close 3 while kick
                    ))),
        new AimRoutine(0.4),
        new ParallelRoutine(
            new SequentialRoutine(
                new ControlKickerRoutine(Shooter.KickerState.KICK, 0.3), // Shoot close 3
                new ParallelRoutine(
                    new IntakeRoutine(1.7),
                    new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.3))),
            new DrivePathRoutine(new WantedTrajectory(p2), 1.1) // Drive close 2 while kick
            ),
        new AimRoutine(0.4),
        new ParallelRoutine(
            new SequentialRoutine(
                new ControlKickerRoutine(Shooter.KickerState.KICK, 0.3), // Shoot close 2
                new ParallelRoutine(
                    new IntakeRoutine(1.7),
                    new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.3))),
            new DrivePathRoutine(new WantedTrajectory(p3), 1.1) // Drive close 1 while kick
            ),
        new AimRoutine(0.4),
        new ParallelRoutine(
            new SequentialRoutine(
                new ControlKickerRoutine(Shooter.KickerState.KICK, 0.3), // Shoot close 1
                new ParallelRoutine(
                    new IntakeRoutine(3.0),
                    new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.3))),
            new DrivePathRoutine(new WantedTrajectory(p4), 1.1) // Drive center 1 while kick
            ),
        new AimRoutine(0.4),
        new ControlKickerRoutine(Shooter.KickerState.KICK, 0.3),
        new ParallelRoutine(
            new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.3),
            new SequentialRoutine(new TimedRoutine(0.7), new IntakeRoutine(1.7)),
            new DrivePathRoutine(new WantedTrajectory(p5), 1.1)),
        new AimRoutine(0.4),
        new ControlKickerRoutine(Shooter.KickerState.KICK, 0.3),
        new ParallelRoutine(
            new ControlKickerRoutine(Shooter.KickerState.HOLD, 0.3),
            new SequentialRoutine(new TimedRoutine(0.7), new IntakeRoutine(1.6)),
            new DrivePathRoutine(new WantedTrajectory(p6), 1.1)));
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
