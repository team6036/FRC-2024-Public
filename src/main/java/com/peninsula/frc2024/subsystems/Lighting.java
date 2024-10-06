package com.peninsula.frc2024.subsystems;

import com.ctre.phoenix.led.Animation;
import com.peninsula.frc2024.config.LightingConstants;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.ReadOnly;
import com.peninsula.frc2024.robot.RobotState;

public class Lighting extends SubsystemBase {

  public enum State {
    IDLE,
    SHOOTING,
    INTAKING,
    HAS_NOTE,
    ALIGNED,
    DIS,
    LOW_BATTERY,
    OFF
  }

  private static Lighting sInstance = new Lighting();
  private State mState;

  private Lighting() {}

  public static Lighting getInstance() {
    return sInstance;
  }

  public State getState() {
    return mState;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    State wantedState = commands.wantedLighting;

    // check for new lighting state
    if (mState != wantedState) {
      mState = wantedState;
      switch (mState) {
        case IDLE -> mOutput = LightingConstants.redSolid;
        case SHOOTING -> mOutput = LightingConstants.greenBlink;
        case ALIGNED -> mOutput = LightingConstants.blueBlink;
        case INTAKING -> mOutput = LightingConstants.goldBlink;
        case HAS_NOTE -> mOutput = LightingConstants.orangeSolid;
        case LOW_BATTERY -> mOutput = LightingConstants.lowBattery;
        case DIS -> mOutput = LightingConstants.rainbow;
      }
    }
  }

  Animation mOutput;

  public Animation getOutput() {
    return mOutput;
  }
}
