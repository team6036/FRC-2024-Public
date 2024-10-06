import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.junit.jupiter.api.Test;

public class FSDTest {

  double epsilon = 1E-3;

  @Test
  public void testFull() {
    ProfiledPIDController t =
        new ProfiledPIDController(
            0.1, 0, 0.01, new TrapezoidProfile.Constraints(3 * Math.PI, Math.PI * 3));
    t.enableContinuousInput(-Math.PI, Math.PI);
    t.reset(Math.PI);

    double z = Math.PI;
    for (int i = 0; i < 400; ++i) {
      //      t.reset(Math.PI + 0.01 * i);
      z += t.calculate(z, Math.PI);
      System.out.println("z: " + z);
    }
  }
}
