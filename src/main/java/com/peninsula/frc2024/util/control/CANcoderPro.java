package com.peninsula.frc2024.util.control;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

public class CANcoderPro extends CANcoder {
  private StatusSignal<Double> mAbsolutePosition = getAbsolutePosition();
  private StatusSignal<Double> mPosition = getPosition();

  public CANcoderPro(int deviceID, String canbus) {
    super(deviceID, canbus);
  }

  public double getAbsolutePositionValue() {
    return mAbsolutePosition.refresh().getValue();
  }

  public double getPositionValue() {
    return mPosition.refresh().getValue();
  }
}
