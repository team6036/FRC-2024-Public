package com.peninsula.frc2024.behavior.routines.statesetters;

import com.peninsula.frc2024.behavior.TimeoutRoutineBase;
import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.ReadOnly;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.subsystems.Arm;
import com.peninsula.frc2024.subsystems.Intake;
import com.peninsula.frc2024.subsystems.Turret;

public class IntakeRoutine extends TimeoutRoutineBase {

  public IntakeRoutine(double timeout) {
    super(timeout);
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    commands.intakeWanted = Intake.State.RUN;
    commands.turretWanted = Turret.State.INTAKE;
    commands.wantedArm = Arm.State.INTAKE;
  }

  @Override
  public boolean checkIfFinishedEarly(RobotState state) {
    return state.pieceInKick;
    //    return false;
  }
}
