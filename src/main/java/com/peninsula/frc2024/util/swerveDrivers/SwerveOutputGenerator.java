package com.peninsula.frc2024.util.swerveDrivers;

import com.peninsula.frc2024.config.SwerveConstants;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.util.GeoUtil;
import com.peninsula.frc2024.util.Util;
import com.peninsula.frc2024.util.control.SwerveOutputs;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
// import com.peninsula.frc2023.util.swerveDrivers.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/** Copied from 254, thx 254 :D I totally appreciate your custom geometry classes */
public class SwerveOutputGenerator {
  public static class KinematicLimits {
    public double kMaxDriveVelocity; // m/s
    public double kMaxDriveAcceleration; // m/s^2
    public double kMaxSteeringVelocity; // rad/s
  }

  private final SwerveDriveKinematics mKinematics;

  public SwerveOutputGenerator(final SwerveDriveKinematics kinematics) {
    this.mKinematics = kinematics;
  }

  /**
   * Check if it would be faster to go to the opposite of the goal heading (and reverse drive
   * direction).
   *
   * @param prevToGoal The rotation from the previous state to the goal state (i.e.
   *     prev.inverse().rotateBy(goal)).
   * @return True if the shortest path to achieve this rotation involves flipping the drive
   *     direction.
   */
  private boolean flipHeading(Rotation2d prevToGoal) {
    return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
  }

  private double unwrapAngle(double ref, double angle) {
    double diff = angle - ref;
    if (diff > Math.PI) {
      return angle - 2.0 * Math.PI;
    } else if (diff < -Math.PI) {
      return angle + 2.0 * Math.PI;
    } else {
      return angle;
    }
  }

  @FunctionalInterface
  private interface Function2d {
    public double f(double x, double y);
  }

  /**
   * Find the root of the generic 2D parametric function 'func' using the regula falsi technique.
   * This is a pretty naive way to do root finding, but it's usually faster than simple bisection
   * while being robust in ways that e.g. the Newton-Raphson method isn't.
   *
   * @param func The Function2d to take the root of.
   * @param x_0 x value of the lower bracket.
   * @param y_0 y value of the lower bracket.
   * @param f_0 value of 'func' at x_0, y_0 (passed in by caller to save a call to 'func' during
   *     recursion)
   * @param x_1 x value of the upper bracket.
   * @param y_1 y value of the upper bracket.
   * @param f_1 value of 'func' at x_1, y_1 (passed in by caller to save a call to 'func' during
   *     recursion)
   * @param iterations_left Number of iterations of root finding left.
   * @return The parameter value 's' that interpolating between 0 and 1 that corresponds to the
   *     (approximate) root.
   */
  private double findRoot(
      Function2d func,
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      int iterations_left) {
    if (iterations_left < 0 || Util.approximatelyEqual(f_0, f_1)) {
      return 1.0;
    }
    var s_guess = Math.max(0.0, Math.min(1.0, -f_0 / (f_1 - f_0)));
    var x_guess = (x_1 - x_0) * s_guess + x_0;
    var y_guess = (y_1 - y_0) * s_guess + y_0;
    var f_guess = func.f(x_guess, y_guess);
    if (Math.signum(f_0) == Math.signum(f_guess)) {
      // 0 and guess on same side of root, so use upper bracket.
      return s_guess
          + (1.0 - s_guess)
              * findRoot(func, x_guess, y_guess, f_guess, x_1, y_1, f_1, iterations_left - 1);
    } else {
      // Use lower bracket.
      return s_guess
          * findRoot(func, x_0, y_0, f_0, x_guess, y_guess, f_guess, iterations_left - 1);
    }
  }

  protected double findSteeringMaxS(
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      double max_deviation,
      int max_iterations) {
    f_1 = unwrapAngle(f_0, f_1);
    double diff = f_1 - f_0;
    if (Math.abs(diff) <= max_deviation) {
      // Can go all the way to s=1.
      return 1.0;
    }
    double offset = f_0 + Math.signum(diff) * max_deviation;
    Function2d func =
        (x, y) -> {
          return unwrapAngle(f_0, Math.atan2(y, x)) - offset;
        };
    return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
  }

  protected double findDriveMaxS(
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      double max_vel_step,
      int max_iterations) {
    double diff = f_1 - f_0;
    if (Math.abs(diff) <= max_vel_step) {
      // Can go all the way to s=1.
      return 1.0;
    }
    double offset = f_0 + Math.signum(diff) * max_vel_step;
    Function2d func =
        (x, y) -> {
          return Math.hypot(x, y) - offset;
        };
    return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
  }

  protected double findDriveMaxS(
      double x_0, double y_0, double x_1, double y_1, double max_vel_step) {
    // Our drive velocity between s=0 and s=1 is quadratic in s:
    // v^2 = ((x_1 - x_0) * s + x_0)^2 + ((y_1 - y_0) * s + y_0)^2
    //     = a * s^2 + b * s + c
    // Where:
    //   a = (x_1 - x_0)^2 + (y_1 - y_0)^2
    //   b = 2 * x_0 * (x_1 - x_0) + 2 * y_0 * (y_1 - y_0)
    //   c = x_0^2 + y_0^2
    // We want to find where this quadratic results in a velocity that is > max_vel_step from our
    // velocity at s=0:
    // sqrt(x_0^2 + y_0^2) +/- max_vel_step = ...quadratic...
    final double dx = x_1 - x_0;
    final double dy = y_1 - y_0;
    final double a = dx * dx + dy * dy;
    final double b = 2.0 * x_0 * dx + 2.0 * y_0 * dy;
    final double c = x_0 * x_0 + y_0 * y_0;
    final double v_limit_upper_2 = Math.pow(Math.hypot(x_0, y_0) + max_vel_step, 2.0);
    final double v_limit_lower_2 = Math.pow(Math.hypot(x_0, y_0) - max_vel_step, 2.0);
    return 0.0;
  }

  public SwerveModuleState[] moduleStatesWithChassisCorrection(ChassisSpeeds wantedChassisSpeeds) {

    double dt = 0.02;

    Pose2d poseDeltaWanted =
        new Pose2d(
            wantedChassisSpeeds.vxMetersPerSecond * dt,
            wantedChassisSpeeds.vyMetersPerSecond * dt,
            new Rotation2d(wantedChassisSpeeds.omegaRadiansPerSecond * dt));
    Pose2d startPoseRelative = new Pose2d(0, 0, new Rotation2d(0));

    Twist2d arcTravel = startPoseRelative.log(poseDeltaWanted);

    ChassisSpeeds correctChassisSpeeds =
        new ChassisSpeeds(arcTravel.dx / dt, arcTravel.dy / dt, arcTravel.dtheta / dt);

    return SwerveConstants.kKinematics.toSwerveModuleStates(correctChassisSpeeds);
  }

  /**
   * Generate a new setpoint.
   *
   * @param limits The kinematic limits to respect for this setpoint.
   * @param prevSetpoint The previous setpoint motion. Normally, you'd pass in the previous
   *     iteration setpoint instead of the actual measured/estimated kinematic state.
   * @param desiredState The desired state of motion, such as from the driver sticks or a path
   *     following algorithm.
   * @param dt The loop time.
   * @return A Setpoint object that satisfies all of the KinematicLimits while converging to
   *     desiredState quickly.
   */
  public SwerveOutputs generateSetpoint(
      final KinematicLimits limits,
      final SwerveOutputs prevSetpoint,
      RobotState state,
      ChassisSpeeds desiredState,
      double dt) {
    //        final Translation2d[] modules = mKinematics.getModuleLocations();
    final Translation2d[] modules = SwerveConstants.getModulePos();
    //        SwerveModuleState[] desiredModuleState =
    // mKinematics.toSwerveModuleStates(desiredState);
    SwerveModuleState[] desiredModuleState = moduleStatesWithChassisCorrection(desiredState);
    // Make sure desiredState respects velocity limits.
    if (limits.kMaxDriveVelocity > 0.0) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleState, limits.kMaxDriveVelocity);
      desiredState = mKinematics.toChassisSpeeds(desiredModuleState);
    }

    // Special case: desiredState is a complete stop. In this case, module angle is arbitrary, so
    // just use the previous angle.
    boolean need_to_steer = true;
    //        if (desiredState.toTwist2d().epsilonEquals(Util.identidyTwist, Util.kEpsilon)) {
    if (GeoUtil.approximatelyEqual(
        GeoUtil.ChassisSpeedToTwist2d(desiredState), GeoUtil.identidyTwist)) {
      need_to_steer = false;
      for (int i = 0; i < modules.length; ++i) {
        //                desiredModuleState[i].angle = prevSetpoint.getStates()[i].angle;
        desiredModuleState[i].angle = prevSetpoint.getStates()[i].angle;
        desiredModuleState[i].speedMetersPerSecond = 0.0;
      }
    }

    // For each module, compute local Vx and Vy vectors.
    double[] prev_vx = new double[modules.length];
    double[] prev_vy = new double[modules.length];
    Rotation2d[] prev_heading = new Rotation2d[modules.length];
    double[] desired_vx = new double[modules.length];
    double[] desired_vy = new double[modules.length];
    Rotation2d[] desired_heading = new Rotation2d[modules.length];
    boolean all_modules_should_flip = true;
    for (int i = 0; i < modules.length; ++i) {
      prev_vx[i] =
          prevSetpoint.getStates()[i].angle.getCos()
              * prevSetpoint.getStates()[i].speedMetersPerSecond;
      prev_vy[i] =
          prevSetpoint.getStates()[i].angle.getSin()
              * prevSetpoint.getStates()[i].speedMetersPerSecond;
      prev_heading[i] = prevSetpoint.getStates()[i].angle;
      if (prevSetpoint.getStates()[i].speedMetersPerSecond < 0.0) {
        prev_heading[i] = GeoUtil.flipRotation2d(prev_heading[i]);
      }
      desired_vx[i] =
          desiredModuleState[i].angle.getCos() * desiredModuleState[i].speedMetersPerSecond;
      desired_vy[i] =
          desiredModuleState[i].angle.getSin() * desiredModuleState[i].speedMetersPerSecond;
      desired_heading[i] = desiredModuleState[i].angle;
      if (desiredModuleState[i].speedMetersPerSecond < 0.0) {
        desired_heading[i] = GeoUtil.flipRotation2d(desired_heading[i]);
      }
      if (all_modules_should_flip) {
        double required_rotation_rad =
            Math.abs(
                GeoUtil.inverseRotation2d(prev_heading[i])
                    .rotateBy(desired_heading[i])
                    .getRadians());
        if (required_rotation_rad < Math.PI / 2.0) {
          all_modules_should_flip = false;
        }
      }
    }
    if (all_modules_should_flip
        && !GeoUtil.approximatelyEqual(
            GeoUtil.ChassisSpeedToTwist2d(prevSetpoint.getChassisSpeeds()), GeoUtil.identidyTwist)
        && !GeoUtil.approximatelyEqual(
            GeoUtil.ChassisSpeedToTwist2d(desiredState), GeoUtil.identidyTwist)) {
      // It will (likely) be faster to stop the robot, rotate the modules in place to the complement
      // of the desired
      // angle, and accelerate again.
      return generateSetpoint(limits, prevSetpoint, state, new ChassisSpeeds(), dt);
    }

    // Compute the deltas between start and goal. We can then interpolate from the start state to
    // the goal state; then
    // find the amount we can move from start towards goal in this cycle such that no kinematic
    // limit is exceeded.
    double dx = desiredState.vxMetersPerSecond - prevSetpoint.getChassisSpeeds().vxMetersPerSecond;
    double dy = desiredState.vyMetersPerSecond - prevSetpoint.getChassisSpeeds().vyMetersPerSecond;
    double dtheta =
        desiredState.omegaRadiansPerSecond - prevSetpoint.getChassisSpeeds().omegaRadiansPerSecond;

    // 's' interpolates between start and goal. At 0, we are at prevState and at 1, we are at
    // desiredState.
    double min_s = 1.0;

    // In cases where an individual module is stopped, we want to remember the right steering angle
    // to command (since
    // inverse kinematics doesn't care about angle, we can be opportunistically lazy).
    List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(modules.length);
    // Enforce steering velocity limits. We do this by taking the derivative of steering angle at
    // the current angle,
    // and then backing out the maximum interpolant between start and goal states. We remember the
    // minimum across all modules, since
    // that is the active constraint.
    final double max_theta_step = dt * limits.kMaxSteeringVelocity;
    for (int i = 0; i < modules.length; ++i) {
      if (!need_to_steer) {
        overrideSteering.add(Optional.of(prevSetpoint.getStates()[i].angle));
        continue;
      }
      overrideSteering.add(Optional.empty());
      if (Util.approximatelyEqual(prevSetpoint.getStates()[i].speedMetersPerSecond, 0.0)) {
        // If module is stopped, we know that we will need to move straight to the final steering
        // angle, so limit based
        // purely on rotation in place.
        if (Util.approximatelyEqual(desiredModuleState[i].speedMetersPerSecond, 0.0)) {
          // Goal angle doesn't matter. Just leave module at its current angle.
          overrideSteering.set(i, Optional.of(prevSetpoint.getStates()[i].angle));
          continue;
        }

        var necessaryRotation =
            GeoUtil.inverseRotation2d(prevSetpoint.getStates()[i].angle)
                .rotateBy(desiredModuleState[i].angle);
        if (flipHeading(necessaryRotation)) {
          necessaryRotation = necessaryRotation.rotateBy(GeoUtil.kPi);
        }
        // getRadians() bounds to +/- Pi.
        final double numStepsNeeded = Math.abs(necessaryRotation.getRadians()) / max_theta_step;

        if (numStepsNeeded <= 1.0) {
          // Steer directly to goal angle.
          overrideSteering.set(i, Optional.of(desiredModuleState[i].angle));
          // Don't limit the global min_s;
          continue;
        } else {
          // Adjust steering by max_theta_step.
          overrideSteering.set(
              i,
              Optional.of(
                  prevSetpoint.getStates()[i].angle.rotateBy(
                      Rotation2d.fromRadians(
                          Math.signum(necessaryRotation.getRadians()) * max_theta_step))));
          min_s = 0.0;
          continue;
        }
      }
      if (min_s == 0.0) {
        // s can't get any lower. Save some CPU.
        continue;
      }

      final int kMaxIterations = 8;
      double s =
          findSteeringMaxS(
              prev_vx[i],
              prev_vy[i],
              prev_heading[i].getRadians(),
              desired_vx[i],
              desired_vy[i],
              desired_heading[i].getRadians(),
              max_theta_step,
              kMaxIterations);
      min_s = Math.min(min_s, s);
    }

    // Enforce drive wheel acceleration limits.
    // TODO, include cg caluclation here. dt * Math.min(cg limit, max drive acceleration)
    final double max_vel_step = dt * limits.kMaxDriveAcceleration;
    for (int i = 0; i < modules.length; ++i) {
      if (min_s == 0.0) {
        // No need to carry on.
        break;
      }
      double vx_min_s =
          min_s == 1.0 ? desired_vx[i] : (desired_vx[i] - prev_vx[i]) * min_s + prev_vx[i];
      double vy_min_s =
          min_s == 1.0 ? desired_vy[i] : (desired_vy[i] - prev_vy[i]) * min_s + prev_vy[i];
      // Find the max s for this drive wheel. Search on the interval between 0 and min_s, because we
      // already know we can't go faster
      // than that.
      // TODO(for efficiency, do all this on v^2 to save a bunch of sqrts)
      // TODO(be smarter about root finding, since this is just a quadratic in s:
      // ((xf-x0)*s+x0)^2+((yf-y0)*s+y0)^2)
      final int kMaxIterations = 10;
      double s =
          min_s
              * findDriveMaxS(
                  prev_vx[i],
                  prev_vy[i],
                  Math.hypot(prev_vx[i], prev_vy[i]),
                  vx_min_s,
                  vy_min_s,
                  Math.hypot(vx_min_s, vy_min_s),
                  max_vel_step,
                  kMaxIterations);
      min_s = Math.min(min_s, s);
    }

    ChassisSpeeds retSpeeds =
        new ChassisSpeeds(
            prevSetpoint.getChassisSpeeds().vxMetersPerSecond + min_s * dx,
            prevSetpoint.getChassisSpeeds().vyMetersPerSecond + min_s * dy,
            prevSetpoint.getChassisSpeeds().omegaRadiansPerSecond + min_s * dtheta);
    var retStates = mKinematics.toSwerveModuleStates(retSpeeds);
    for (int i = 0; i < modules.length; ++i) {
      final var maybeOverride = overrideSteering.get(i);
      if (maybeOverride.isPresent()) {
        var override = maybeOverride.get();
        if (flipHeading(GeoUtil.inverseRotation2d(retStates[i].angle).rotateBy(override))) {
          retStates[i].speedMetersPerSecond *= -1.0;
        }
        retStates[i].angle = override;
      }
      final var deltaRotation =
          GeoUtil.inverseRotation2d(prevSetpoint.getStates()[i].angle).rotateBy(retStates[i].angle);
      if (flipHeading(deltaRotation)) {
        retStates[i].angle = GeoUtil.flipRotation2d(retStates[i].angle);
        retStates[i].speedMetersPerSecond *= -1.0;
      }
    }
    // TODO, Our SwerveOutputs accepts module states only. The chassis speed is used for calculation
    // in the next cycle.
    // retSpeeds should be stored in RobotState unless chassisStates constantly updated.
    //        return new SwerveSetpoint(retSpeeds, retStates);
    return new SwerveOutputs(retStates, retSpeeds);
  }
}
