package com.peninsula.frc2024.subsystems;

import com.peninsula.frc2024.config.ArmConstants;
import com.peninsula.frc2024.config.ShootingConstants;
import com.peninsula.frc2024.logging.Logger;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.util.control.ControllerOutput;
import com.peninsula.frc2024.util.control.ProfiledGains;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemBase {

  public enum State {
    SHOOTING_AIM,
    ANGLE,
    INTAKE,
    AMP,
    SOURCE,
    CLIMBED,
    CLIMB,
    HOARD_SHOT,
    TRAP
  }

  private static final Arm sInstance = new Arm();
  private State mState = State.SHOOTING_AIM;

  ControllerOutput mOutput_big = new ControllerOutput();

  public static Arm getInstance() {
    return sInstance;
  }

  public State getState() {
    return mState;
  }

  public ControllerOutput getOutputsBig() {
    return mOutput_big;
  }

  @Override
  public void update(Commands commands, RobotState state) {

    State wantedState = commands.wantedArm;

    if (mState != wantedState) {
      mState = wantedState;
    }

    switch (mState) {
      case INTAKE:
        mOutput_big.setTargetPositionProfiled(
            ArmConstants.intake * ArmConstants.pivotGearRatio, new ProfiledGains());
        break;
      case AMP:
        mOutput_big.setTargetPositionProfiled(
            (ShootingConstants.ampDirect ? ArmConstants.ampNum.get() : ArmConstants.amp_shoot)
                * ArmConstants.pivotGearRatio,
            new ProfiledGains());
        break;
      case SOURCE:
        mOutput_big.setTargetPositionProfiled(
            (ArmConstants.source) * ArmConstants.pivotGearRatio, new ProfiledGains());
        break;
      case SHOOTING_AIM:
        mOutput_big.setTargetPositionProfiled(
            (ShootingConstants.shooterMap
                        .get(state.virtual_goal.minus(state.lastEst).getTranslation().getNorm())
                        .angle()
                    + state.shooterTrim)
                * ArmConstants.pivotGearRatio,
            new ProfiledGains());
        //        mOutput_big.setTargetPositionProfiled(ArmConstants.shooterAngle.get() *
        // ArmConstants.pivotGearRatio, new ProfiledGains());
        break;
      case ANGLE:
        mOutput_big.setTargetPositionProfiled(
            commands.commandedAngle * ArmConstants.pivotGearRatio, new ProfiledGains());
        break;
      case CLIMBED:
        mOutput_big.setTargetPositionProfiled(
            ArmConstants.climbed * ArmConstants.pivotGearRatio, new ProfiledGains());
        break;
      case CLIMB:
        mOutput_big.setTargetPositionProfiled(
            ArmConstants.climb * ArmConstants.pivotGearRatio, new ProfiledGains());
        break;
      case HOARD_SHOT:
        mOutput_big.setTargetPositionProfiled(
            (ShootingConstants.hoardMap
                        .get(
                            state
                                .hoard_virtual_goal
                                .minus(state.lastEst)
                                .getTranslation()
                                .getNorm())
                        .angle()
                    + state.shooterTrim)
                * ArmConstants.pivotGearRatio,
            new ProfiledGains());
        break;
      case TRAP:
        double trapAngle = ArmConstants.trapNum.get();
        if (trapAngle > 0.25) trapAngle = 0;
        mOutput_big.setTargetPositionProfiled(
            trapAngle * ArmConstants.pivotGearRatio, new ProfiledGains());
        break;
    }

    SmartDashboard.putNumber(
        "distance to goal", state.virtual_goal.minus(state.lastEst).getTranslation().getNorm());

    SmartDashboard.putNumber(
        "distance to hoard",
        state.hoard_virtual_goal.minus(state.lastEst).getTranslation().getNorm());

    Logger.getInstance()
        .log(
            "Vision/Distance to goal",
            state.virtual_goal.minus(state.lastEst).getTranslation().getNorm());
  }
}
