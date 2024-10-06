import static org.junit.jupiter.api.Assertions.assertEquals;

import com.peninsula.frc2024.util.Util;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.PeninsulaTrajectory;
import java.io.IOException;
import org.junit.jupiter.api.Test;

public class PeninsulaTrajectoryReaderTest {
  @Test
  public void trajectoryReadTest() throws IOException {
    String file = "trajectories/BasicTrajectory.json";

    PeninsulaTrajectory traj = PeninsulaTrajectory.of(file);

    assertEquals(traj.dt, 0.02, Util.kEpsilon);

    PeninsulaTrajectory.TrajectoryState state =
        (PeninsulaTrajectory.TrajectoryState) traj.getStates().get(0);

    assertEquals(state.poseMeters.getX(), -2.6462408275435002e-21, Util.kEpsilon);

    state = (PeninsulaTrajectory.TrajectoryState) traj.getStates().get(4);

    assertEquals(state.velo_x, 1.4934241599851283, Util.kEpsilon);
  }

  @Test
  public void trajectoryInterpolationTest() throws IOException {
    String file = "trajectories/BasicTrajectory.json";

    PeninsulaTrajectory traj = PeninsulaTrajectory.of(file);

    assertEquals(traj.sample(0.00).velo_x, -5.2952898669822275e-22, Util.kEpsilon);
    assertEquals(
        traj.sample(traj.getTotalTimeSeconds()).poseMeters.getRotation().getRadians(),
        3.1429051740642233,
        Util.kEpsilon);

    assertEquals(traj.sample(0.01).timeSeconds, 0.01, Util.kEpsilon);
    assertEquals(traj.sample(0.02).poseMeters.getX(), 0.004514569549556587, Util.kEpsilon);
    assertEquals(traj.sample(0.03).velo_y, 0.12695796096, Util.kEpsilon);
  }
}
