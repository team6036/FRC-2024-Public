package com.peninsula.frc2024.auto.drive;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.peninsula.frc2024.auto.AutoBase;
import com.peninsula.frc2024.behavior.RoutineBase;
import com.peninsula.frc2024.behavior.SequentialRoutine;
import com.peninsula.frc2024.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2024.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2024.robot.Robot;
import com.peninsula.frc2024.util.Flipper;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.PeninsulaTrajectory;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.WantedTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.io.IOException;

public class CombinedTest implements AutoBase {
  @Override
  public RoutineBase getRoutine() {
    PathPlannerPath f = PathPlannerPath.fromPathFile("Path2");
    PathPlannerTrajectory d = new PathPlannerTrajectory(f, new ChassisSpeeds(), new Rotation2d());

    PeninsulaTrajectory grab_1_return = null;
    PeninsulaTrajectory grab_2 = null;
    PeninsulaTrajectory return_from_2 = null;
    PeninsulaTrajectory grab_3_return = null;
    try {
      grab_1_return = PeninsulaTrajectory.of("trajectories/OutAndInObstacle/outandin/data.json");
      grab_2 = PeninsulaTrajectory.of("trajectories/CubePlaceSecondPieceInteraction/out/data.json");
      return_from_2 =
          PeninsulaTrajectory.of("trajectories/CubePlaceSecondPieceInteraction/in/data.json");
      grab_3_return = PeninsulaTrajectory.of("trajectories/UTurn/outandin/data.json");
    } catch (IOException e) {
      throw new RuntimeException(e); // L
    }

    Pose2d inital = d.getInitialState().getTargetHolonomicPose();

    if (Robot.onBlueAlliance) {
      inital = Flipper.flip(inital);
    }

    return new SequentialRoutine(
        new DriveSetOdometryRoutine(
            inital.getX(), inital.getY(), inital.getRotation().getDegrees()),
        new DrivePathRoutine(new WantedTrajectory(d), 1.025),
        new DrivePathRoutine(new WantedTrajectory(grab_1_return), 1.025),
        new DrivePathRoutine(new WantedTrajectory(grab_2), 1.025),
        new DrivePathRoutine(new WantedTrajectory(return_from_2), 1.025),
        new DrivePathRoutine(new WantedTrajectory(grab_3_return), 1.025));
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
