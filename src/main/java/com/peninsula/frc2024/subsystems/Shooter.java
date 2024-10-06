package com.peninsula.frc2024.subsystems;

import com.peninsula.frc2024.config.ShooterConstants;
import com.peninsula.frc2024.config.ShootingConstants;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.util.ShotMap;
import com.peninsula.frc2024.util.control.ControllerOutput;
import com.peninsula.frc2024.util.control.Gains;

public class Shooter extends SubsystemBase {

  public enum State {
    SET_SPEED,
    SLOW,
    OFF,
    IDLE,
    AMP,
    HOARD_SHOT,
    SOURCE,
    TRAP
  }

  public enum KickerState {
    KICK,
    KICK_HARD,
    HOLD,
    SOURCE,
  }

  public enum BlowerState {
    ON,
    OFF
  }

  public enum SourceIntakeStage {
    INTAKE,
    INTAKE_NOTE_DETECTED,
    HOLD
  }

  private ControllerOutput rightWheel = new ControllerOutput();
  private ControllerOutput leftWheel = new ControllerOutput();
  public static Shooter instance = new Shooter();

  private ControllerOutput kicker = new ControllerOutput();

  private double blowerOutPercentage = 0.0;

  private Shooter() {}

  public static Shooter getInstance() {
    return instance;
  }

  public ControllerOutput getOutputs1() {
    return rightWheel;
  }

  public ControllerOutput getOutputs2() {
    return leftWheel;
  }

  public ControllerOutput getOutputsKicker() {
    return kicker;
  }

  public double getBlowerOutPercentage() {
    return blowerOutPercentage;
  }

  public void update(Commands commands, RobotState state) {
    switch (commands.getShooterWanted()) {
      case SET_SPEED:
        ShotMap.ShotData shot =
            ShootingConstants.shooterMap.get(
                state.virtual_goal.minus(state.lastEst).getTranslation().getNorm());
        rightWheel.setTargetVelocity(shot.rpmRight(), new Gains());
        leftWheel.setTargetVelocity(shot.rpmLeft(), new Gains());
        //        rightWheel.setTargetVelocity(ShooterConstants.shooterRight.get(), new Gains());
        //        leftWheel.setTargetVelocity(ShooterConstants.shooterLeft.get(), new Gains());
        break;
      case OFF:
        rightWheel.setIdle();
        leftWheel.setIdle();
        break;
      case SLOW:
        rightWheel.setTargetVelocity(ShooterConstants.close.getFirst(), new Gains());
        leftWheel.setTargetVelocity(ShooterConstants.close.getSecond(), new Gains());
        break;
      case IDLE:
        rightWheel.setPercentOutput(ShooterConstants.idlePercentOut);
        leftWheel.setPercentOutput(ShooterConstants.idlePercentOut);
        break;
      case AMP:
        rightWheel.setTargetVelocity(ShooterConstants.amp.getFirst(), new Gains());
        leftWheel.setTargetVelocity(ShooterConstants.amp.getFirst(), new Gains());
        break;
      case HOARD_SHOT:
        ShotMap.ShotData hoardShot =
            ShootingConstants.hoardMap.get(
                state.hoard_virtual_goal.minus(state.lastEst).getTranslation().getNorm());
        rightWheel.setTargetVelocity(hoardShot.rpmRight(), new Gains());
        leftWheel.setTargetVelocity(hoardShot.rpmLeft(), new Gains());
        break;
      case SOURCE:
        rightWheel.setTargetVelocity(ShooterConstants.source.getFirst(), new Gains());
        leftWheel.setTargetVelocity(ShooterConstants.source.getSecond(), new Gains());
        break;
      case TRAP:
        rightWheel.setTargetVelocity(ShooterConstants.trapVelo.get(), new Gains());
        leftWheel.setTargetVelocity(ShooterConstants.trapVelo.get(), new Gains());
        break;
    }

    switch (commands.kickerWanted) {
      case HOLD:
        if (state.pieceInKick) {
          kicker.setBrake();
        } else if (state.backSensor) {
          kicker.setTargetVelocity(ShooterConstants.kickerRunVel, new Gains());
        } else {
          kicker.setTargetVelocity(ShooterConstants.kickerFastVel, new Gains());
        }
        break;
      case KICK:
        kicker.setTargetVelocity(ShooterConstants.kickerRunVel, new Gains());
        break;
      case KICK_HARD:
        kicker.setTargetVelocity(ShooterConstants.kickerRunVel, new Gains());
        break;
      case SOURCE:
        kicker.setTargetVelocity(ShooterConstants.kickerSourceVel, new Gains());
        break;
    }

    switch (commands.blowerWanted) {
      case OFF -> blowerOutPercentage = 0.0;
      case ON -> blowerOutPercentage = -1.0;
    }
  }
}
