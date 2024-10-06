package com.peninsula.frc2024.subsystems;

import com.peninsula.frc2024.robot.Commands;
import com.peninsula.frc2024.robot.RobotState;
import com.peninsula.frc2024.vision.LimelightHelpers;
import com.peninsula.frc2024.vision.ProjectionHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ejml.simple.SimpleMatrix;

public class Limelight extends SubsystemBase {

  public enum State {
    ON,
    OFF
  }

  public SimpleMatrix noteLocation;

  private static final Limelight sInstance = new Limelight();

  public static Limelight getInstance() {
    return sInstance;
  }

  @Override
  public void update(Commands commands, RobotState state) {
    State wantedState = state.limelightWanted;

    switch (wantedState) {
      case ON:
        if (commands.limelightOff) {
          noteLocation = new SimpleMatrix(new double[] {0, 0, 0});
          SmartDashboard.putString("Note Location", noteLocation.toString());
        } else {
          if (state.limelightResults == null) {
            noteLocation = new SimpleMatrix(new double[] {0, 0, 0});
            break;
          }

          if (state.limelightResults.targetingResults.targets_Detector.length > 0) {
            LimelightHelpers.LimelightTarget_Detector detection =
                state.limelightResults.targetingResults.targets_Detector[0];

            double pixel_x = detection.tx_pixels, pixel_y = detection.ty_pixels;

            noteLocation = ProjectionHelpers.project(pixel_x, pixel_y);
            SmartDashboard.putNumber("note_location_x", noteLocation.get(0));
            SmartDashboard.putNumber("note_location_y", noteLocation.get(1));
            SmartDashboard.putNumber("pixel_x", pixel_x);
            SmartDashboard.putNumber("pixel_y", pixel_y);
            break;
          }

          noteLocation = new SimpleMatrix(new double[] {0, 0, 0});
          break;
        }
        break;

      case OFF:
        noteLocation = new SimpleMatrix(new double[] {0, 0, 0});
        break;
    }
  }
}
