package com.peninsula.frc2024.behavior.routines.statesetters;

import com.peninsula.frc2024.behavior.TimeoutRoutineBase;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.ReadOnly;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.Arm;
import com.peninsula.frc2024.subsystems.Intake;
import com.peninsula.frc2024.subsystems.Turret;

public class AimRoutine extends TimeoutRoutineBase {

  Commands d;

  public AimRoutine(double timeout) {
    super(timeout);
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    commands.turretWanted = Turret.State.AIM_SHOOT_ON_MOVE;
    commands.wantedArm = Arm.State.SHOOTING_AIM;
    commands.intakeWanted = Intake.State.OFF;
  }

  @Override
  public boolean checkIfFinishedEarly(RobotState state) {
    return false;
  }
}
