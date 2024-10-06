package com.peninsula.frc2024.util.peninsulaCoolios.phronesis.fieldAwareness;

import com.peninsula.frc2024.config.FieldConstants;
import edu.wpi.first.math.geometry.Pose2d;

public class FieldObserver {
  public void addPieceSeenObservationAuto(Pose2d piece_pose, FieldState state, double timestamp) {
    // Check if in a staged zone
    if (piece_pose.getX() < FieldConstants.GamePieceLocations.blueWingLineX) {
      for (int i = 0; i < 3; i++) {
        double distance =
            piece_pose
                .minus(FieldConstants.GamePieceLocations.blueWing[i])
                .getTranslation()
                .getNorm();
        if (distance < FieldConstants.GamePieceLocations.wingSeparationY * 0.5) {
          state.blue_wing[i] = true;
          state.blue_wing_observations[i] = new FieldState.NoteObservation(timestamp, piece_pose);
          break;
        }
      }
    } else if (piece_pose.getX() > FieldConstants.GamePieceLocations.redWingLineX) {
      for (int i = 0; i < 3; i++) {
        double distance =
            piece_pose
                .minus(FieldConstants.GamePieceLocations.redWing[i])
                .getTranslation()
                .getNorm();
        if (distance < FieldConstants.GamePieceLocations.wingSeparationY * 0.5) {
          state.red_wing[i] = true;
          state.red_wing_observations[i] = new FieldState.NoteObservation(timestamp, piece_pose);
          break;
        }
      }
    } else {
      for (int i = 0; i < 5; i++) {
        double distance =
            piece_pose
                .minus(FieldConstants.GamePieceLocations.centerline[i])
                .getTranslation()
                .getNorm();
        if (distance < FieldConstants.GamePieceLocations.centerlineSpacing * 0.5) {
          state.center[i] = true;
          state.center_observations[i] = new FieldState.NoteObservation(timestamp, piece_pose);
          break;
        }
      }
    }

    // Add to unstaged with repeat check
    addUnstagedNote(piece_pose, state, timestamp);
  }

  public void addUnstagedNote(Pose2d piece_pose, FieldState state, double timestamp) {
    cleanStaleNoteObservations(state, timestamp);

    // Check if already in array
    for (int i = 0; i < state.notesSeen.size(); i++) {
      if (piece_pose.minus(state.notesSeen.get(i).location()).getTranslation().getNorm()
          < FieldConstants.GamePieceLocations.sameNoteTolerance) {
        break;
      }
    }

    state.notesSeen.addFirst(new FieldState.NoteObservation(timestamp, piece_pose));
  }

  public void cleanStaleNoteObservations(FieldState state, double timestamp) {
    while (state.notesSeen.size() > 0) {
      FieldState.NoteObservation observation = state.notesSeen.getLast();
      if (timestamp - observation.time() > FieldConstants.GamePieceLocations.noteStaleTimeSeconds)
        state.notesSeen.removeLast();
      else break;
    }
  }
}
