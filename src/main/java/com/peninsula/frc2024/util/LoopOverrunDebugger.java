package com.peninsula.frc2024.util;

import com.peninsula.frc2024.logging.Logger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;

public class LoopOverrunDebugger {

  private static class Measurement {

    String name;
    double durationSeconds;

    Measurement(String name, double durationSeconds) {
      this.name = name;
      this.durationSeconds = durationSeconds;
    }
  }

  public Timer mTimer = new Timer();
  private final ArrayList<Measurement> mMeasurements = new ArrayList<>(8);

  public LoopOverrunDebugger(String name, double printDurationSeconds) {
    this(name);
  }

  public LoopOverrunDebugger(String name) {
    mTimer.start();
  }

  public void addPoint(String name) {
    mMeasurements.add(new Measurement(name, mTimer.get()));
  }

  public void finish() {
    double loopTime = mTimer.get() * 1000.0;
    SmartDashboard.putNumber("loop time ms", loopTime);
    Logger.getInstance().log("loop time ms", loopTime);
    printSummary();
  }

  private void printSummary() {
    mMeasurements.add(0, new Measurement("Start", 0));
    for (int i = 1; i < mMeasurements.size(); ++i) {
      double meas =
          (mMeasurements.get(i).durationSeconds - mMeasurements.get(i - 1).durationSeconds) * 1000;

      SmartDashboard.putNumber("LoopSummary/" + mMeasurements.get(i).name, meas);

      Logger.getInstance().log("LoopSummary/" + mMeasurements.get(i).name, meas);
    }
  }

  public void reset() {
    mTimer.reset();
    mMeasurements.clear();
  }
}
