package com.peninsula.frc2024.subsystems;

import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.vision.PerceptionObservation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.stream.Collectors;

public class Vision extends SubsystemBase {

  public enum State {
    ON,
    OFF
  }

  private static final Vision sInstance = new Vision();
  private static double lastMultiTagSeen = Timer.getFPGATimestamp();

  private ArrayList<PerceptionObservation> mVisionOutputs = new ArrayList<>();

  public static Vision getInstance() {
    return sInstance;
  }

  public ArrayList<PerceptionObservation> getVisionOutputs() {
    return mVisionOutputs;
  }

  @Override
  public void update(Commands commands, RobotState state) {
    ArrayList<PerceptionObservation> observations = state.perceptionResults;
    State wantedState = state.visionWanted;

    double currentTime = Timer.getFPGATimestamp();
    for (PerceptionObservation o : observations) {
      if (o.getNumTags() > 1) {
        lastMultiTagSeen = currentTime;
        break;
      }
    }
    SmartDashboard.putBoolean("Vision/MultiTag Seen", currentTime - lastMultiTagSeen < 0.1);

    switch (wantedState) {
      case ON:
        if (commands.visionOff) {
          mVisionOutputs = new ArrayList<>();
        } else {
          mVisionOutputs =
              observations.stream()
                  .filter(this::filterObservations)
                  .collect(Collectors.toCollection(ArrayList::new));
          SmartDashboard.putNumber("Vision/Poses Applied", state.perceptionResults.size());
        }
        break;

      case OFF:
        mVisionOutputs = new ArrayList<>();
        break;
    }
  }

  public boolean filterObservations(PerceptionObservation observation) {
    if (observation.getPose().getRotation().getX() > 0.1
        && observation.getPose().getRotation().getY() > 0.1) {
      return false;
    }
    if (observation.getPose().getZ() > 0.5 || observation.getPose().getZ() < -0.5) {
      return false;
    }
    //    if (observation.getNumTags() == 1) {
    //    if (Timer.getFPGATimestamp() - lastMultiTagSeen < 0.1 && observation.getNumTags() == 1)
    // {
    //      return false;
    //    }
    return true;
  }
}
