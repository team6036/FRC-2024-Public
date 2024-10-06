package com.peninsula.frc2024.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.peninsula.frc2024.config.PortConstants;
import com.peninsula.frc2024.config.SwerveConstants;
import com.peninsula.frc2024.config.VisionConstants;
import com.peninsula.frc2024.util.config.TalonFXFactory;
import com.peninsula.frc2024.util.control.TalonFXController;
import com.peninsula.frc2024.util.swerveDrivers.SwerveModule;
import com.peninsula.frc2024.vision.PerceptionCamera;
import edu.wpi.first.wpilibj.*;

/**
 * Represents all hardware components of the robot. Singleton class. Should only be used in robot
 * package. Subdivides hardware into subsystems.
 */
public class HardwareAdapter {

  /**
   * 4 Falcon 500s (controlled by Talon FX), 1 Pigeon IMU Gyro connected via Talon SRX data cable.
   */
  public static class SwerveHardware {

    private static SwerveHardware sInstance;

    // CW Starting from top left module
    // <Drive Motor, Turn Motor, Encoder>
    public final SwerveModule FL, FR, BL, BR;
    public final SwerveModule[] modules;

    public final Pigeon2 gyro;

    private SwerveHardware() {

      FL = new SwerveModule(0, SwerveConstants.Constants.Swerve.Mod0.constants);
      FR = new SwerveModule(1, SwerveConstants.Constants.Swerve.Mod1.constants);
      BL = new SwerveModule(2, SwerveConstants.Constants.Swerve.Mod2.constants);
      BR = new SwerveModule(3, SwerveConstants.Constants.Swerve.Mod3.constants);
      modules = new SwerveModule[] {FL, FR, BL, BR};

      gyro = new Pigeon2(PortConstants.kPigeon, "swerve");
    }

    static SwerveHardware getInstance() {
      if (sInstance == null) sInstance = new SwerveHardware();
      return sInstance;
    }
  }

  public static class VisionHardware {

    private static VisionHardware sInstance;
    public final PerceptionCamera cameraBackLeft;
    public final PerceptionCamera cameraBackRight;
    public final PerceptionCamera cameraFrontLeft;
    public final PerceptionCamera cameraFrontRight;
    PerceptionCamera[] cameras;

    private VisionHardware() {

      cameraBackRight = new PerceptionCamera("back_right");
      cameraBackRight.setTransform(VisionConstants.robot_to_camera_back_right.inverse());

      cameraBackLeft = new PerceptionCamera("back_left");
      cameraBackLeft.setTransform(VisionConstants.robot_to_camera_back_left.inverse());

      cameraFrontRight = new PerceptionCamera("front_right");
      cameraFrontRight.setTransform(VisionConstants.robot_to_camera_front_right.inverse());

      cameraFrontLeft = new PerceptionCamera("front_left");
      cameraFrontLeft.setTransform(VisionConstants.robot_to_camera_front_left.inverse());

      cameras =
          new PerceptionCamera[] {
            cameraFrontRight, cameraFrontLeft, cameraBackRight, cameraBackLeft
          };
    }

    static VisionHardware getInstance() {
      if (sInstance == null) sInstance = new VisionHardware();
      return sInstance;
    }
  }

  public static class LimelightHardware {
    private static LimelightHardware sInstance;

    private LimelightHardware() {
      // do nothing
    }

    static LimelightHardware getInstance() {
      if (sInstance == null) sInstance = new LimelightHardware();

      return sInstance;
    }
  }

  /** 22 Xbox Controller */
  static class JoystickHardware {

    private static JoystickHardware sInstance;

    final XboxController driverXboxController;
    final XboxController operatorXboxController;

    private JoystickHardware() {
      driverXboxController = new XboxController(PortConstants.kDriverId);
      operatorXboxController = new XboxController(PortConstants.kOperatorId);
    }

    static JoystickHardware getInstance() {
      if (sInstance == null) sInstance = new JoystickHardware();
      return sInstance;
    }
  }

  static class LightingHardware {
    private static LightingHardware sInstance;

    final CANdle candle;

    private LightingHardware() {
      candle = new CANdle(PortConstants.kLighting);
    }

    static LightingHardware getInstance() {
      if (sInstance == null) sInstance = new LightingHardware();
      return sInstance;
    }
  }

  static class IntakeHardware {
    private static IntakeHardware sInstance;

    final TalonFXController motor;

    private IntakeHardware() {
      motor = TalonFXFactory.createDefaultFalconProFOC(PortConstants.kIntakeId, "intake");
    }

    static IntakeHardware getInstance() {
      if (sInstance == null) sInstance = new IntakeHardware();
      return sInstance;
    }
  }

  static class ShooterHardware {
    private static ShooterHardware sInstance;

    final TalonFXController rightMotor;
    final TalonFXController leftMotor;

    final TalonFXController kicker;

    final TalonSRX blower;

    final DigitalInput sensor;
    final DigitalInput backKickerSensor;

    final SynchronousInterrupt sensorInterrupt;

    private ShooterHardware() {
      rightMotor =
          TalonFXFactory.createDefaultFalconProFOC(PortConstants.kRight, "shooter R", "turret");
      leftMotor =
          TalonFXFactory.createDefaultFalconProFOC(PortConstants.kLeft, "shooter L", "turret");

      kicker = TalonFXFactory.createDefaultFalconPro(PortConstants.kKicker, "kicker", "turret");

      blower = new TalonSRX(2);

      sensor = new DigitalInput(PortConstants.kSensor);
      backKickerSensor = new DigitalInput(PortConstants.kBackSensor);

      sensorInterrupt = new SynchronousInterrupt(sensor);
    }

    static ShooterHardware getInstance() {
      if (sInstance == null) sInstance = new ShooterHardware();
      return sInstance;
    }
  }

  static class TurretHardware {
    private static TurretHardware sInstance;
    final TalonFXController motor;
    final CANcoder absoluteEncoder;
    final StatusSignal<Double> absolutePosition;

    private TurretHardware() {
      motor = TalonFXFactory.createDefaultFalconPro(PortConstants.turretMotor, "TurretMotor");
      absoluteEncoder = new CANcoder(PortConstants.turretEncoder, "swerve");
      absolutePosition = absoluteEncoder.getAbsolutePosition();
    }

    static TurretHardware getInstance() {
      if (sInstance == null) sInstance = new TurretHardware();
      return sInstance;
    }
  }

  static class ArmHardware {
    private static ArmHardware sInstance;

    final TalonFXController motorCeo;
    final TalonFXController motorEmployee1, motorEmployee2;

    private ArmHardware() {
      motorCeo =
          TalonFXFactory.createDefaultFalconPro(PortConstants.kPivotCeo, "ARM CEO", "turret");

      motorEmployee1 =
          TalonFXFactory.createDefaultFalconPro(
              PortConstants.kPivotEmployee1, "ARM EMPLOYEE 1", "turret");
      motorEmployee1.setControl(new Follower(PortConstants.kPivotCeo, false));
      motorEmployee2 =
          TalonFXFactory.createDefaultFalconPro(
              PortConstants.kPivotEmployee2, "ARM EMPLOYEE 2", "turret");
      motorEmployee2.setControl(new Follower(PortConstants.kPivotCeo, true));
    }

    static ArmHardware getInstance() {
      if (sInstance == null) sInstance = new ArmHardware();
      return sInstance;
    }
  }

  private HardwareAdapter() {}
}
