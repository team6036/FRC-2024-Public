package com.peninsula.frc2024.util.peninsulaCoolios.phronesis.fieldAwareness;

import com.peninsula.frc2024.config.FieldConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldState {

  public enum PieceType {
    WING,
    CENTERLINE,
    NONE
  }

  public boolean[] red_wing = new boolean[3];
  public NoteObservation[] red_wing_observations = new NoteObservation[3];

  public boolean[] blue_wing = new boolean[3];
  public NoteObservation[] blue_wing_observations = new NoteObservation[3];

  public boolean[] center = new boolean[5];
  public NoteObservation[] center_observations = new NoteObservation[5];

  public CircularBuffer<NoteObservation> notesSeen = new CircularBuffer<>(20);

  public FieldState() {
    for (int i = 0; i < 3; i++) {
      red_wing_observations[i] =
          new NoteObservation(0, FieldConstants.GamePieceLocations.redWing[i]);
      blue_wing_observations[i] =
          new NoteObservation(0, FieldConstants.GamePieceLocations.blueWing[i]);
    }

    for (int i = 0; i < 5; i++) {
      center_observations[i] =
          new NoteObservation(0, FieldConstants.GamePieceLocations.centerline[i]);
    }
  }

  public boolean seenNoteSince(
      PieceType type, DriverStation.Alliance color, int number, double timestamp) {
    if (type == PieceType.WING) {
      if (color == DriverStation.Alliance.Red)
        return red_wing_observations[number - 1].time >= timestamp;
      else return blue_wing_observations[number - 1].time >= timestamp;
    }

    SmartDashboard.putNumber("c S" + (number - 1), center_observations[number - 1].time);
    SmartDashboard.putNumber("C T", timestamp);

    return center_observations[number - 1].time >= timestamp;
  }

  public boolean seeingNote(PieceType type, DriverStation.Alliance color, int number) {
    return seenNoteSince(
        type,
        color,
        number,
        Timer.getFPGATimestamp() - FieldConstants.GamePieceLocations.noteSeeingTimeEpsilon);
  }

  public record NoteObservation(double time, Pose2d location) {}
}
