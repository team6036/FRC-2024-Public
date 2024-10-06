package com.peninsula.frc2024.util.control;

import java.util.Objects;

public class ProfiledGains extends Gains {

  public double acceleration, velocity, allowableError, minimumOutputVelocity;

  public ProfiledGains() {}

  public ProfiledGains(
      double p,
      double i,
      double d,
      double f,
      double iZone,
      double iMax,
      double acceleration,
      double velocity,
      double allowableError,
      double minimumOutputVelocity) {
    super(p, i, d, f, iZone, iMax);
    this.acceleration = acceleration;
    this.velocity = velocity;
    this.allowableError = allowableError;
    this.minimumOutputVelocity = minimumOutputVelocity;
  }

  @Override // Auto-generated
  public int hashCode() {
    return Objects.hash(
        super.hashCode(), acceleration, velocity, allowableError, minimumOutputVelocity);
  }

  @Override // Auto-generated
  public boolean equals(Object other) {
    if (this == other) return true;
    if (!(other instanceof ProfiledGains)) return false;
    if (!super.equals(other)) return false;
    ProfiledGains otherGains = (ProfiledGains) other;
    return Double.compare(otherGains.acceleration, acceleration) == 0
        && Double.compare(otherGains.velocity, velocity) == 0
        && Double.compare(otherGains.allowableError, allowableError) == 0
        && Double.compare(otherGains.minimumOutputVelocity, minimumOutputVelocity) == 0;
  }

  @Override
  public String toString() { // Auto-generated
    return String.format(
        "ProfiledGains{acceleration=%f, velocity=%f, allowableError=%f, minimumOutputVelocity=%f}%n%s",
        acceleration, velocity, allowableError, minimumOutputVelocity, super.toString());
  }
}
