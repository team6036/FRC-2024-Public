package com.peninsula.frc2024.util;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class TrajectoryReader {
  private final double deltaTime;
  private final List<Double> trajectoryP1;
  private final List<Double> trajectoryP2;

  public TrajectoryReader(String json) throws IOException {
    this.trajectoryP1 = new ArrayList<>();
    this.trajectoryP2 = new ArrayList<>();

    BufferedReader reader = new BufferedReader(new FileReader(json));

    reader.readLine(); // initial bracket
    String dt = reader.readLine().strip();
    this.deltaTime = Double.parseDouble(dt.substring(6, dt.length() - 1));

    reader.readLine();

    while (!(reader.readLine().strip()).equals("]")) {
      String j1 = reader.readLine().strip(), j2 = reader.readLine().strip();
      reader.readLine();
      reader.readLine();
      reader.readLine(); // skip next 3 useless lines

      trajectoryP1.add(Double.parseDouble(j1.substring(6, j1.length() - 1)));
      trajectoryP2.add(Double.parseDouble(j2.substring(6, j2.length() - 1)));
    }

    reader.close();
  }

  public double[] getTrajectoryP1() {
    double[] j1 = new double[trajectoryP1.size()];
    for (int i = 0; i < trajectoryP1.size(); i++) j1[i] = trajectoryP1.get(i);
    return j1;
  }

  public double[] getTrajectoryP2() {
    double[] j2 = new double[trajectoryP2.size()];
    for (int i = 0; i < trajectoryP2.size(); i++) j2[i] = trajectoryP2.get(i);
    return j2;
  }

  public double getDeltaTime() {
    return this.deltaTime;
  }
}
