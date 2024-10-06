package com.peninsula.frc2024.util.config;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.peninsula.frc2024.util.control.TalonFXController;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

public class TalonFXFactory {

  private static final int kTimeoutMs = 100;

  // create a CANTalon with the default (out of the box) configuration
  public static TalonFXController createFalconPro(
      int id, TalonFXConfiguration config, String name, String canbus, boolean FOC) {
    TalonFXController talon = new TalonFXController(id, name, canbus, FOC);

    applyAndCheckConfiguration(talon, config, 5);
    talon.clearStickyFaults();

    return talon;
  }

  public static boolean checkErrorAndRetry(Supplier<StatusCode> function, int numTries) {
    StatusCode code = function.get();
    int tries = 0;
    while (code != StatusCode.OK && tries < numTries) {
      DriverStation.reportWarning("Retrying CTRE Device Config " + code.getName(), false);
      code = function.get();
      tries++;
    }
    if (code != StatusCode.OK) {
      DriverStation.reportError(
          "Failed to execute phoenix pro api call after " + numTries + " attempts", false);
      return false;
    }
    return true;
  }

  public static boolean applyAndCheckConfiguration(
      TalonFXController talon, TalonFXConfiguration config, int numTries) {
    for (int i = 0; i < numTries; i++) {
      if (checkErrorAndRetry(() -> talon.getConfigurator().apply(config, 0.5), 5)) {
        // API says we applied config, lets make sure it's right
        if (readAndVerifyConfiguration(talon, config)) {
          return true;
        } else {
          DriverStation.reportWarning(
              "Failed to verify config for talon ["
                  + talon.getDescription()
                  + "] (attempt "
                  + (i + 1)
                  + " of "
                  + numTries
                  + ")",
              false);
        }
      } else {
        DriverStation.reportWarning(
            "Failed to apply config for talon ["
                + talon.getDescription()
                + "] (attempt "
                + (i + 1)
                + " of "
                + numTries
                + ")",
            false);
      }
    }
    DriverStation.reportError(
        "Failed to apply config for talon after " + numTries + " attempts", false);
    return false;
  }

  public static boolean readAndVerifyConfiguration(
      TalonFXController talon, TalonFXConfiguration config) {
    TalonFXConfiguration readConfig = new TalonFXConfiguration();
    if (!checkErrorAndRetry(() -> talon.getConfigurator().refresh(readConfig), 5)) {
      // could not get config!
      DriverStation.reportWarning(
          "Failed to read config for talon [" + talon.getDescription() + "]", false);
      return false;
    } else if (!TalonConfigEquality.isEqual(config, readConfig)) {
      // configs did not match
      DriverStation.reportWarning(
          "Configuration verification failed for talon [" + talon.getDescription() + "]", false);
      return false;
    } else {
      // configs read and match, Talon OK
      return true;
    }
  }

  public static TalonFXController createDefaultFalconPro(int id, String name) {
    return createFalconPro(id, new TalonFXConfiguration(), name, "swerve", false);
  }

  public static TalonFXController createDefaultFalconPro(int id, String name, String canbus) {
    return createFalconPro(id, new TalonFXConfiguration(), name, canbus, false);
  }

  public static TalonFXController createDefaultFalconProFOC(int id, String name) {
    return createFalconPro(id, new TalonFXConfiguration(), name, "swerve", true);
  }

  public static TalonFXController createDefaultFalconProFOC(int id, String name, String canbus) {
    return createFalconPro(id, new TalonFXConfiguration(), name, canbus, true);
  }
}
