package com.peninsula.frc2024.util.control;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFXController extends TalonFX {

  private String mName;
  private double lastRef = -381904; // random value
  private VelocityVoltage velocityTorqueCurrentFOCSetter = new VelocityVoltage(0);
  private final PositionVoltage positionVoltageSetter = new PositionVoltage(0);
  private final MotionMagicVoltage motionMagicTorqueCurrentFOCSetter = new MotionMagicVoltage(0);
  private final NeutralOut neutralOutSetter = new NeutralOut();
  private VoltageOut voltageOutSetter = new VoltageOut(0);

  private final StatusSignal<Double> positionStatus = getPosition();
  private final StatusSignal<Double> velocityStatus = getRotorVelocity();

  private boolean foc = false;

  public TalonFXController(int deviceNumber, String name, String canbus, boolean FOC) {
    super(deviceNumber, canbus);
    this.mName = name;

    this.foc = FOC;

    this.voltageOutSetter = new VoltageOut(0).withEnableFOC(FOC);
    this.velocityTorqueCurrentFOCSetter = new VelocityVoltage(0).withEnableFOC(FOC);
  }

  public TalonFXController(int deviceNumber, String name, String canbus) {
    super(deviceNumber, canbus);
    this.mName = name;
  }

  public TalonFXController(int deviceNumber, String name) {
    super(deviceNumber);
    this.mName = name;
  }

  public void setOutput(ControllerOutput outputs, boolean resetGains) {

    ControllerOutput.Mode mode = outputs.getControlMode();
    double demand = outputs.getArbitraryDemand();
    Gains gains = outputs.getGains();
    double reference = outputs.getReference();

    if (gains != null && resetGains) {
      setGain(gains);
    }

    switch (mode) {
      case VELOCITY:
      case PROFILED_VELOCITY:
        setControl(velocityTorqueCurrentFOCSetter.withVelocity(reference)); // RPS
        //        setControl(velocityTorqueCurrentFOCSetter.withVelocity(reference)); // RPS
        break;
      case PERCENT_OUTPUT:
        set(reference);
        break;
      case POSITION:
        setControl(positionVoltageSetter.withPosition(reference)); // Rotation
        break;
      case PROFILED_POSITION:
        setControl(
            motionMagicTorqueCurrentFOCSetter.withPosition(reference).withFeedForward(demand));
        break;
      case BRAKE:
        setControl(neutralOutSetter);
        break;
      case VOLTAGE:
        setControl(voltageOutSetter.withOutput(reference));
        //        setControl(voltageOutSetter.withOutput(reference / 12.0));
        //        setControl(dutyCycleOutSetter.withOutput(reference / 12.0));
        break;
    }
    lastRef = reference;
  }

  public String getName() {
    return mName;
  }

  /**
   * Sets the target velocity for motion magic
   *
   * @param sensorUnitsPer100ms Velocity in rotation of target velocity
   */
  public void setMotionCruiseVelocity(double sensorUnitsPer100ms) {
    MotionMagicConfigs m = new MotionMagicConfigs();
    m.MotionMagicCruiseVelocity = sensorUnitsPer100ms;
    getConfigurator().apply(m);
  }

  /**
   * Sets the target acceleration for motion magic
   *
   * @param sensorUnitsPer100ms Acceleration in rotation of target velocity
   */
  public void setMotionAcceleration(double sensorUnitsPer100ms) {
    MotionMagicConfigs m = new MotionMagicConfigs();
    m.MotionMagicAcceleration = sensorUnitsPer100ms;
    getConfigurator().apply(m);
  }

  /**
   * Sets the target velocity and acceleration for motion magic. <br>
   * Same as calling setMotionCruiseVelocity and setMotionAcceleration in effect, but creates 1 less
   * MotionMagicConfigs object
   *
   * @param acceleration Acceleration in rotation of target velocity
   * @param velocity Velocity in rotation of target velocity
   */
  public void setMotionAccelerationAndVelocity(double acceleration, double velocity) {
    MotionMagicConfigs m = new MotionMagicConfigs();
    m.MotionMagicAcceleration = acceleration;
    m.MotionMagicCruiseVelocity = velocity;
    getConfigurator().apply(m);
  }

  /**
   * Sets the PIDF of the motor, applies to slot0 of configurator
   *
   * @param gains
   */
  public void setGain(Gains gains) {
    Slot0Configs configs = new Slot0Configs();
    configs.kP = gains.p;
    configs.kI = gains.i;
    configs.kD = gains.d;
    configs.kV = gains.f;

    getConfigurator().apply(configs, 50);
  }

  /**
   * Set a master motor. Slave motor follows the motions of master motor. <br>
   * Same thing as setControl(new Follower(master.getDeviceID(), setInverse));
   *
   * @param master Motor to follow
   * @param setInverse If slave motor inverse of master.
   */
  public void follow(TalonFXController master, boolean setInverse) {
    setControl(new Follower(master.getDeviceID(), setInverse));
  }

  /**
   * Refreshes the rotor position status signal object and returns the value. DOES NOT construct a
   * new status signal object. <br>
   * </br> Same as: rotorPositionStatus.refresh().getValue();
   *
   * @return Rotor Position in rotations
   */
  public double getPositionValue() {
    return positionStatus.refresh().getValue();
  }

  /**
   * Refreshes the rotor velocity status signal object and returns the value. DOES NOT construct a
   * new status signal object. <br>
   * </br> Same as: rotorVelocityStatus.refresh().getValue();
   *
   * @return Rotor Velocity in rotations per second
   */
  public double getRotorVelocityValue() {
    return velocityStatus.refresh().getValue();
  }
}
