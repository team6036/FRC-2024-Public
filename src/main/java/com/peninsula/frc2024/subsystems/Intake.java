package com.peninsula.frc2024.subsystems;

import com.peninsula.frc2024.config.IntakeConstants;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.util.control.ControllerOutput;

public class Intake extends SubsystemBase {

  public enum State {
    RUN,
    OFF,
    EJECT
  }

  private Intake() {}

  private ControllerOutput mOutputs = new ControllerOutput();
  private static Intake sInstance = new Intake();

  public static Intake getInstance() {
    return sInstance;
  }

  public ControllerOutput getOutputs() {
    return mOutputs;
  }

  public void update(Commands commands, RobotState state) {
    switch (commands.intakeWanted) {
      case RUN:
        mOutputs.setVolt(IntakeConstants.kIntakeInPercentOutput * 12);
        break;
      case OFF:
        mOutputs.setIdle();
        break;
      case EJECT:
        mOutputs.setVolt(IntakeConstants.kIntakeEjectPercentOutput * 12);
        //        mOutputs.setPercentOutput(IntakeConstants.kIntakeEjectPercentOutput);
        break;
    }
  }
}
