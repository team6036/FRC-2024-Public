import static org.junit.jupiter.api.Assertions.assertEquals;

import com.peninsula.frc2024.util.Util;
import org.junit.jupiter.api.Test;

public class TestHandleDeadBand {

  private static final double MAX_DELTA = 0.001;
  private static final double deadband = 0.2;

  @Test
  public void testHandleDeadBand_ZeroValue() {
    assertEquals(0.0, Util.handleDeadBand(0.0, deadband), MAX_DELTA);
  }

  @Test
  public void testHandleDeadBand_UnderDeadbandPositive() {
    assertEquals(0.0, Util.handleDeadBand(0.1, deadband), MAX_DELTA);
  }

  @Test
  public void testHandleDeadBand_UnderDeadbandNegative() {
    assertEquals(0.0, Util.handleDeadBand(-0.1, deadband), MAX_DELTA);
  }

  @Test
  public void testHandleDeadBand_EdgeOfDeadbandPositive() {
    assertEquals(0.0, Util.handleDeadBand(deadband, deadband), MAX_DELTA);
  }

  @Test
  public void testHandleDeadBand_EdgeOfDeadbandNegative() {
    assertEquals(0.0, Util.handleDeadBand(-deadband, deadband), MAX_DELTA);
  }

  @Test
  public void testHandleDeadBand_JustOverDeadbandPositive() {
    assertEquals(
        (0.3 - deadband) / (1.0 - deadband), Util.handleDeadBand(0.3, deadband), MAX_DELTA);
  }

  @Test
  public void testHandleDeadBand_JustOverDeadbandNegative() {
    assertEquals(
        (-0.3 + deadband) / (1.0 - deadband), Util.handleDeadBand(-0.3, deadband), MAX_DELTA);
  }

  @Test
  public void testHandleDeadBand_MaxValuePositive() {
    assertEquals(1.0, Util.handleDeadBand(1.0, deadband), MAX_DELTA);
  }

  @Test
  public void testHandleDeadBand_MaxValueNegative() {
    assertEquals(-1.0, Util.handleDeadBand(-1.0, deadband), MAX_DELTA);
  }

  @Test
  public void testHandleDeadBand_MiddleValuePositive() {
    assertEquals(
        (0.5 - deadband) / (1.0 - deadband), Util.handleDeadBand(0.5, deadband), MAX_DELTA);
  }

  @Test
  public void testHandleDeadBand_MiddleValueNegative() {
    assertEquals(
        (-0.5 + deadband) / (1.0 - deadband), Util.handleDeadBand(-0.5, deadband), MAX_DELTA);
  }
}
