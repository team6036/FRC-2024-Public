import static org.junit.jupiter.api.Assertions.assertEquals;

import com.peninsula.frc2024.util.peninsulaCoolios.BilinearInterpolationMap;
import java.util.Arrays;
import org.junit.jupiter.api.Test;

public class TestBilinear {

  @Test
  public void bilinearTest() {
    double[][] test = {
      {1, 2, 3},
      {4, 5, 6},
      {7, 8, 9}
    };

    System.out.println(Arrays.deepToString(test));

    BilinearInterpolationMap map = new BilinearInterpolationMap(test, 2);

    System.out.println("Index 0.5, 0.5: " + map.getIndex(0.5, 0.5));
    System.out.println("Meters 1.0, 1.0: " + map.getMeters(1, 1));

    assertEquals(map.getIndex(0.5, 0.5), 3.0, 0.0);
    assertEquals(map.getMeters(1.0, 1.0), 3.0, 0.0);
  }

  @Test
  public void testContinuous() {
    BilinearInterpolationMap map =
        new BilinearInterpolationMap(
            new double[][] {
              {0.0, 0.0, 00.0},
              {120.0, 120.0, 340.0},
              {0.0, 0.0, 0.0}
            },
            4);

    assertEquals(map.getMeters(3.9999, 3), map.getMeters(4.0001, 3), 0.1);
  }
}
