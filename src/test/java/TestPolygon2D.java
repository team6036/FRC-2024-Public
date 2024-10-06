import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.peninsula.frc2024.util.peninsulaCoolios.Polygon2D;
import org.junit.jupiter.api.Test;

public class TestPolygon2D {
  @Test
  public void polygonPIPSquareTest() {
    double[] x = {0, 1, 1, 0};
    double[] y = {0, 0, 1, 1};
    Polygon2D polygon2D = new Polygon2D(x, y);

    // Inside cases
    assertTrue(polygon2D.PIP(0.5, 0.5));

    // Outside cases
    assertFalse(polygon2D.PIP(1.5, 0.5));
    assertFalse(polygon2D.PIP(-0.5, 0.5));

    // On the line
    assertTrue(polygon2D.PIP(1, 0.5));
  }

  @Test
  public void polygonPIPTriangleTest() {
    double[] x = {0, 1, 1};
    double[] y = {0, 0, 1};
    Polygon2D polygon2D = new Polygon2D(x, y);

    // Inside cases
    assertTrue(polygon2D.PIP(0.5, 0.25));
    assertTrue(polygon2D.PIP(0.5, 0.5));

    // Outside cases
    assertFalse(polygon2D.PIP(0.5, 0.6));
  }
}
