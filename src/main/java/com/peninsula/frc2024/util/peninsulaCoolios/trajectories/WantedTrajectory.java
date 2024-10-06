package com.peninsula.frc2024.util.peninsulaCoolios.trajectories;

import com.pathplanner.lib.path.PathPlannerTrajectory;

public class WantedTrajectory {
  public enum Type {
    PATH_PLANNER,
    PENINSULA
  }

  private Type type;

  private PathPlannerTrajectory pathPlannerTrajectory;
  private PeninsulaTrajectory peninsulaTrajectory;

  public WantedTrajectory(PathPlannerTrajectory trajectory) {
    type = Type.PATH_PLANNER;
    this.pathPlannerTrajectory = trajectory;
  }

  public WantedTrajectory(PeninsulaTrajectory trajectory) {
    type = Type.PENINSULA;
    this.peninsulaTrajectory = trajectory;
  }

  public Type getType() {
    return this.type;
  }

  public Object getTrajectory() {
    if (type == Type.PATH_PLANNER) {
      return pathPlannerTrajectory;
    } else {
      return peninsulaTrajectory;
    }
  }
}
