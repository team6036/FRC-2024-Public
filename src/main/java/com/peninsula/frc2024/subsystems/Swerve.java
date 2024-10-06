package com.peninsula.frc2024.subsystems;

import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.ReadOnly;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.controllers.drive.*;
import com.peninsula.frc2024.util.control.SwerveOutputs;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Represents the drivetrain. Uses {@link #mController} to generate {@link #mOutputs}. */
public class Swerve extends SubsystemBase {

  public enum State {
    NEUTRAL,
    TELEOP,
    ALIGN,
    AUTO,
    FSD,
    LANE_KEEP
  }

  public abstract static class SwerveController {

    protected SwerveOutputs mOutputs = new SwerveOutputs();

    public SwerveController(SwerveOutputs mOutputs) {
      this.mOutputs = mOutputs;
    }

    public final SwerveOutputs update(@ReadOnly Commands commands, @ReadOnly RobotState state) {

      updateSignal(commands, state);
      return mOutputs;
    }

    /** Should set {@link #mOutputs} to reflect what is currently wanted by {@link Commands}. */
    public abstract void updateSignal(@ReadOnly Commands commands, @ReadOnly RobotState state);
  }

  private static Swerve sInstance = new Swerve();
  private SwerveController mController;
  private State mState = State.NEUTRAL;
  private SwerveOutputs mOutputs = new SwerveOutputs();
  private boolean mZeroReq = false;

  public boolean getZero() {
    return mZeroReq;
  }

  private Swerve() {}

  public static Swerve getInstance() {
    return sInstance;
  }

  public SwerveOutputs getOutputs() {
    return mOutputs;
  }

  public State getState() {
    return mState;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    State wantedState = commands.swerveWanted;

    boolean isNewState = mState != wantedState;
    mState = wantedState;
    if (isNewState) {
      switch (wantedState) {
        case NEUTRAL:
          mController = null;
          break;
        case TELEOP:
          mController = new TeleopController(mOutputs);
          break;
        case AUTO:
          mController = new AutoDriveController(mOutputs);
          break;
        case FSD:
          mController = new FSDController(mOutputs);
          break;
        case LANE_KEEP:
          mController = new LaneKeepController(mOutputs);
          break;
      }
    }

    if (mController == null) {
      mOutputs.setIdle(true);
    } else {
      mOutputs.setIdle(false);
      mOutputs = mController.update(commands, state);
    }
  }

  /** Sets the output of swerve to the measured modules states. Only use for initialization. */
  public void setOutputFromMeasured(SwerveModuleState[] moduleStates) {
    mOutputs.setOutputs(moduleStates);
  }
}
