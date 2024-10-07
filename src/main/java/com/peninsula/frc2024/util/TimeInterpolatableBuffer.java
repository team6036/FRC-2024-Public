// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.peninsula.frc2024.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import java.util.NavigableMap;
import java.util.TreeMap;

/**
 * The TimeInterpolatableBuffer provides an easy way to estimate past measurements. One application
 * might be in conjunction with the DifferentialDrivePoseEstimator, where knowledge of the robot
 * pose at the time when vision or other global measurement were recorded is necessary, or for
 * recording the past angles of mechanisms as measured by encoders.
 *
 * @param <T> The type stored in this buffer.
 */
public class TimeInterpolatableBuffer<T> {

  private final double m_historySize;
  private final InterpolateFunction<T> m_interpolatingFunc;
  private final NavigableMap<Double, T> m_buffer = new TreeMap<>();

  public TimeInterpolatableBuffer(
      InterpolateFunction<T> interpolateFunction, double historySizeSeconds) {
    this.m_historySize = historySizeSeconds;
    this.m_interpolatingFunc = interpolateFunction;
  }

  /**
   * Create a new TimeInterpolatableBuffer.
   *
   * @param interpolateFunction The function used to interpolate between values.
   * @param historySizeSeconds The history size of the buffer.
   * @param  <T> The type of data to store in the buffer.
   * @return The new TimeInterpolatableBuffer.
   */
  public static <T> TimeInterpolatableBuffer<T> createBuffer(
      InterpolateFunction<T> interpolateFunction, double historySizeSeconds) {
    return new TimeInterpolatableBuffer<>(interpolateFunction, historySizeSeconds);
  }

  /**
   * Create a new TimeInterpolatableBuffer that stores a given subclass of {@link Interpolatable}.
   *
   * @param historySizeSeconds The history size of the buffer.
   * @param  <T> The type of {@link Interpolatable} to store in the buffer.
   * @return The new TimeInterpolatableBuffer.
   */
  public static <T extends Interpolatable<T>> TimeInterpolatableBuffer<T> createBuffer(
      double historySizeSeconds) {
    return new TimeInterpolatableBuffer<>(Interpolatable::interpolate, historySizeSeconds);
  }

  /**
   * Create a new TimeInterpolatableBuffer to store Double values.
   *
   * @param historySizeSeconds The history size of the buffer.
   * @return The new TimeInterpolatableBuffer.
   */
  public static TimeInterpolatableBuffer<Double> createDoubleBuffer(double historySizeSeconds) {
    return new TimeInterpolatableBuffer<>(MathUtil::interpolate, historySizeSeconds);
  }

  /**
   * Add a sample to the buffer.
   *
   * @param timeSeconds The timestamp of the sample.
   * @param sample The sample object.
   */
  public void addSample(double timeSeconds, T sample) {
    cleanUp(timeSeconds);
    m_buffer.put(timeSeconds, sample);
  }

  /**
   * Removes samples older than our current history size.
   *
   * @param time The current timestamp.
   */
  private void cleanUp(double time) {
    while (!m_buffer.isEmpty()) {
      var entry = m_buffer.firstEntry();
      if (time - entry.getKey() >= m_historySize) {
        m_buffer.remove(entry.getKey());
      } else {
        return;
      }
    }
  }

  /** Clear all old samples. */
  public void clear() {
    m_buffer.clear();
  }

  /**
   * Sample the buffer at the given time. If the buffer is empty, this will return null.
   *
   * @param timeSeconds The time at which to sample.
   * @return The interpolated value at that timestamp. Might be null.
   */
  @SuppressWarnings("UnnecessaryParentheses")
  public T getSample(double timeSeconds) {
    if (m_buffer.isEmpty()) {
      return null;
    }

    // Special case for when the requested time is the same as a sample
    var nowEntry = m_buffer.get(timeSeconds);
    if (nowEntry != null) {
      return nowEntry;
    }

    var topBound = m_buffer.ceilingEntry(timeSeconds);
    var bottomBound = m_buffer.floorEntry(timeSeconds);

    // Return null if neither sample exists, and the opposite bound if the other is null
    if (topBound == null && bottomBound == null) {
      return null;
    } else if (topBound == null) {
      return bottomBound.getValue();
    } else if (bottomBound == null) {
      return topBound.getValue();
    } else {
      // Otherwise, interpolate. Because T is between [0, 1], we want the ratio of (the difference
      // between the current time and bottom bound) and (the difference between top and bottom
      // bounds).
      return m_interpolatingFunc.interpolate(
          bottomBound.getValue(),
          topBound.getValue(),
          ((timeSeconds - bottomBound.getKey()) / (topBound.getKey() - bottomBound.getKey())));
    }
  }

  public interface InterpolateFunction<T> {

    /**
     * Return the interpolated value. This object is assumed to be the starting position, or lower
     * bound.
     *
     * @param start The lower bound, or start.
     * @param end The upper bound, or end.
     * @param t How far between the lower and upper bound we are. This should be bounded in [0, 1].
     * @return The interpolated value.
     */
    @SuppressWarnings("ParameterName")
    T interpolate(T start, T end, double t);
  }
}
