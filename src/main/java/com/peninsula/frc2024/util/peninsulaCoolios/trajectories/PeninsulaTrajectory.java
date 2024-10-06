package com.peninsula.frc2024.util.peninsulaCoolios.trajectories;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.peninsula.frc2024.util.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

/**
 * Reads robot trajectories(PeninsulaPlanner) There is a possible issue down the line where the json
 * changes, requiring changes to the substring stuff.
 */
public class PeninsulaTrajectory extends Trajectory {
  public static class TrajectoryState extends Trajectory.State {
    // no additional methods for now
    @JsonProperty public double velo_x;
    @JsonProperty public double velo_y;
    @JsonProperty public double theta_dot;

    public TrajectoryState() {
      this(0, 0, 0, 0, 0, 0);
    }

    public TrajectoryState(
        double x, double y, double theta, double velo_x, double velo_y, double time) {
      this(x, y, theta, velo_x, velo_y, 0, time);
    }

    public TrajectoryState(
        double x,
        double y,
        double theta,
        double velo_x,
        double velo_y,
        double theta_dot,
        double time) {
      this(new Pose2d(x, y, new Rotation2d(theta)), velo_x, velo_y, theta_dot, time);
    }

    public TrajectoryState(
        Pose2d pose, double velo_x, double velo_y, double theta_dot, double time) {
      super(time, mag(velo_x, velo_y), 0, pose, 0); // do we even need accel and curvature?
      this.velo_x = velo_x;
      this.velo_y = velo_y;
      this.theta_dot = theta_dot;
    }

    // this can make setting theta_dot using lambda streaming easy
    public void setThetaDot(double theta_dot) {
      this.theta_dot = theta_dot;
    }

    private static double mag(double v_x, double v_y) {
      return Math.sqrt(v_x * v_x + v_y + v_y);
    }

    @Override
    public String toString() {
      return String.format(
          "TrajectoryState(Sec: %.2f, Vel_X m/s: %.2f, Vel_Y m/s: %.2f, Vel_Mag: %.2f, Theta_Dot: %.2f, Accel m/s/s: %.2f, Pose: %s, Curvature: %.2f)",
          timeSeconds,
          velo_x,
          velo_y,
          velocityMetersPerSecond,
          theta_dot,
          accelerationMetersPerSecondSq,
          poseMeters,
          curvatureRadPerMeter);
    }

    @Override
    public boolean equals(Object obj) {
      if (this == obj) return true;
      if (!(obj instanceof TrajectoryState s)) return false;

      return Double.compare(s.velo_x, velo_x) == 0
          && Double.compare(s.velo_y, velo_y) == 0
          && Double.compare(s.theta_dot, theta_dot) == 0
          && super.equals(obj);
    }

    @Override
    public int hashCode() {
      return Objects.hash(velo_x, velo_y, theta_dot, super.hashCode());
    }
  }

  public double dt;

  // make private
  private PeninsulaTrajectory(double dt, List<TrajectoryState> state) {
    super(Collections.unmodifiableList(state));
    this.dt = dt;
  }

  public static PeninsulaTrajectory of(String fileName) throws IOException {
    File file = new File(Filesystem.getDeployDirectory(), fileName);

    ObjectMapper mapper = new ObjectMapper();

    JsonNode root = mapper.readTree(file);

    double dt = root.path("dt").asDouble();

    JsonNode states = root.path("state");

    double currTime = 0;

    List<TrajectoryState> trajStates = new ArrayList<>();

    for (JsonNode state : states) {
      double x = state.path("x").asDouble();
      double y = state.path("y").asDouble();
      double theta = state.path("theta").asDouble();
      double vx = state.path("vx").asDouble();
      double vy = state.path("vy").asDouble();

      trajStates.add(new TrajectoryState(x, y, theta, vx, vy, currTime));
      currTime += dt;
    }

    return new PeninsulaTrajectory(dt, trajStates);
  }

  @Override
  public TrajectoryState sample(double time) {
    if (time <= 0) {
      return (TrajectoryState) getStates().get(0);
    }

    if (time >= getTotalTimeSeconds()) {
      return (TrajectoryState) getStates().get(getStates().size() - 1);
    }

    // I wish this was C++ because I could use std::lower_bound :yawning:
    int left = 1, right = getStates().size() - 1;

    while (left < right) {
      int mid = left + (right - left) / 2; // can't have that integer overflow

      if (getStates().get(mid).timeSeconds < time) {
        left = mid + 1;
      } else {
        right = mid;
      }
    }

    double progress = (time - getStates().get(left - 1).timeSeconds) / dt;

    return lerp(
        (TrajectoryState) getStates().get(left - 1),
        (TrajectoryState) getStates().get(left),
        progress);
  }

  private TrajectoryState lerp(TrajectoryState a, TrajectoryState b, double t) {
    Pose2d lerped_pose = Util.lerp(a.poseMeters, b.poseMeters, t);
    double lerped_v_x = Util.lerp(a.velo_x, b.velo_x, t),
        lerped_v_y = Util.lerp(a.velo_y, b.velo_y, t);
    double lerped_theta_dot = Util.lerp(a.theta_dot, b.theta_dot, t),
        lerped_time = Util.lerp(a.timeSeconds, b.timeSeconds, t);

    return new TrajectoryState(lerped_pose, lerped_v_x, lerped_v_y, lerped_theta_dot, lerped_time);
  }
}
