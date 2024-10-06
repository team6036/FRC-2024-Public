import com.peninsula.frc2024.vision.ProjectionHelpers;
import java.io.*;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

public class TestProjectionHelpers {
  private static final DecimalFormat FORMATTER = new DecimalFormat("#.#####");

  private static double cost(double angle, double[] pixels, double[] yCoords) {
    double[] results = new double[pixels.length];

    for (int i = 0; i < pixels.length; i++) {
      results[i] = ProjectionHelpers.project(640, pixels[i], angle).get(1);
    }

    double cost = 0;

    for (int i = 0; i < results.length; i++) {
      cost += Math.abs(results[i] - yCoords[i]);
    }

    return cost;
  }

  @Test
  public void getBestAngle() {
    double[] pixels = {591, 454, 306.5, 233, 185, 151, 123, 106, 87, 75.};
    double[] yCoords = {
      0.4572, 0.6096, 0.9114, 1.2192, 1.524, 1.8288, 2.1336, 2.4384, 2.7432, 3.048
    };

    double left = 20, right = 30;

    double epsilon = 1E-4;

    // ternary search
    while (right - left > epsilon) {
      double left_third = left + (right - left) / 3.;
      double right_third = right - (right - left) / 3.;

      if (cost(left_third, pixels, yCoords) > cost(right_third, pixels, yCoords)) {
        left = left_third;
      } else {
        right = right_third;
      }
    }

    double bestAngle = (left + right) / 2;

    System.out.println("Best Angle: " + bestAngle);
    System.out.println(cost(bestAngle, pixels, yCoords));

    for (int i = 0; i < pixels.length; i++) {
      System.out.println(
          "Expected: "
              + yCoords[i]
              + ", Actual: "
              + ProjectionHelpers.project(640, pixels[i], bestAngle).get(1));
    }
  }

  @Test
  public void projectionAccuracyTest() {
    // test all the data we gathered yesterday
    try {
      BufferedReader reader =
          new BufferedReader(new FileReader("src/test/java/note_data_sheet.csv"));

      List<Double>[] values = new ArrayList[4];

      for (int i = 0; i < 4; i++) {
        values[i] = new ArrayList<>();
      }

      reader.readLine(); // csv header

      while (reader.ready()) {
        String[] vals = reader.readLine().split(","); // x, y, pixel_x, pixel_y

        for (int i = 0; i < 4; i++) {
          values[i].add(Double.parseDouble(vals[i]));
        }
      }

      for (int i = 0; i < values[0].size(); i++) {
        SimpleMatrix expected =
            new SimpleMatrix(
                new double[] {values[0].get(i) * 0.0254, values[1].get(i) * 0.0254, 0});
        SimpleMatrix actual = ProjectionHelpers.project(values[2].get(i), values[3].get(i));

        System.out.println(
            "Expected: "
                + ProjectionHelpers.simpleMatrixString(expected)
                + ", Actual: "
                + ProjectionHelpers.simpleMatrixString(actual));

        double xDiff = actual.get(0) - expected.get(0), yDiff = actual.get(1) - expected.get(1);

        System.out.println(
            "X diff: " + FORMATTER.format(xDiff) + ", Y diff: " + FORMATTER.format(yDiff));
        System.out.println("error: " + FORMATTER.format(Math.hypot(xDiff, yDiff)));
      }
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  @Test
  public void projectionTest() {
    // (0.3048, 1.2192, 0)
    System.out.println(
        "Projection Results: "
            + ProjectionHelpers.simpleMatrixString(ProjectionHelpers.project(940, 246.5)));
  }
}
