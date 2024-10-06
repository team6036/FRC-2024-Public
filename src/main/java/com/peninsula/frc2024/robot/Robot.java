package com.peninsula.frc2024.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.peninsula.frc2024.auto.AutoBase;
import com.peninsula.frc2024.auto.drive.*;
import com.peninsula.frc2024.auto.drive.SevenPieceCenterRush;
import com.peninsula.frc2024.behavior.RoutineBase;
import com.peninsula.frc2024.behavior.RoutineManager;
import com.peninsula.frc2024.config.LightingConstants;
import com.peninsula.frc2024.config.RobotConstants;
import com.peninsula.frc2024.logging.Logger;
import com.peninsula.frc2024.logging.shims.LoggedTimedRobot;
import com.peninsula.frc2024.subsystems.*;
import com.peninsula.frc2024.subsystems.SubsystemBase;
import com.peninsula.frc2024.util.LoopOverrunDebugger;
import com.peninsula.frc2024.util.swerveDrivers.SwerveModule;
import com.peninsula.frc2024.vision.PerceptionCamera;
import com.sun.management.GarbageCollectionNotificationInfo;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.IOException;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.*;
import javax.management.Notification;
import javax.management.NotificationEmitter;
import javax.management.NotificationListener;
import javax.management.openmbean.CompositeData;

@SuppressWarnings("java:S1104")
public class Robot extends LoggedTimedRobot {

  public static final double kPeriod = 0.02;

  public static boolean onBlueAlliance = false;

  SendableChooser<AutoBase> autoChooser = new SendableChooser<>();
  RoutineBase autoRoutine;
  ShuffleboardTab tab = Shuffleboard.getTab("Auto chooser");

  Timer autoTimer = new Timer();

  private final RobotState mRobotState = new RobotState();
  private final Control mOperatorInterface = new Control();
  private final RoutineManager mRoutineManager = new RoutineManager();
  private final HardwareReader mHardwareReader = new HardwareReader();
  private final OdometryThread mOdometryThread = new OdometryThread(mRobotState);
  private final HardwareWriter mHardwareWriter = new HardwareWriter();
  private final Commands mCommands = new Commands();

  /* Subsystems */
  private final Vision mVision = Vision.getInstance();
  private final Swerve mSwerve = Swerve.getInstance();
  private final Lighting mLights = Lighting.getInstance();
  private final Shooter mShooter = Shooter.getInstance();
  private final Intake mIntake = Intake.getInstance();
  private final Arm mArm = Arm.getInstance();
  private final Turret mTurret = Turret.getInstance();

  boolean setup = false;

  static {
    //     notification listener. is notified whenever a gc finishes.
    NotificationListener notificationListener =
        new NotificationListener() {
          @Override
          public void handleNotification(Notification notification, Object handback) {
            if (notification
                .getType()
                .equals(GarbageCollectionNotificationInfo.GARBAGE_COLLECTION_NOTIFICATION)) {
              //           extract garbage collection information from notification.
              GarbageCollectionNotificationInfo gcInfo =
                  GarbageCollectionNotificationInfo.from(
                      (CompositeData) notification.getUserData());

              //           access garbage collection information...
              SmartDashboard.putNumber("LoopSummary/gc time", gcInfo.getGcInfo().getDuration());
            }
          }
        };

    //     register our listener with all gc beans
    for (GarbageCollectorMXBean gcBean : ManagementFactory.getGarbageCollectorMXBeans()) {
      NotificationEmitter emitter = (NotificationEmitter) gcBean;
      emitter.addNotificationListener(notificationListener, null, null);
    }
  }

  public static boolean real = RobotBase.isReal();

  public static boolean isRobotReal() {
    return real;
  }

  private final Set<SubsystemBase> mSubsystems =
      switch (RobotConstants.robotType) {
        case COMP -> Set.of(mSwerve, mLights, mShooter, mIntake, mArm, mTurret);
        case PRACTICE -> Set.of(mSwerve);
      };

  public static final LoopOverrunDebugger sLoopDebugger =
      new LoopOverrunDebugger("teleop", kPeriod);

  public Robot() {
    super(kPeriod);
  }

  @Override
  public void robotInit() {
    SignalLogger.enableAutoLogging(false);
    mHardwareWriter.configureHardware(mSubsystems);
    HardwareAdapter.SwerveHardware.getInstance().gyro.setYaw(0);
    if (!isRobotReal()) {
      // Used so that swerve module states are not null in simulation
      Swerve.getInstance()
          .getOutputs()
          .setOutputs(
              new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
              });
    }

    mOdometryThread.setDaemon(true);

    mOdometryThread.start();

    distribution.clearStickyFaults();

    RobotController.setBrownoutVoltage(5.75);
  }

  @Override
  public void simulationInit() {}

  @Override
  public void disabledInit() {
    mRobotState.gamePeriod = RobotState.GamePeriod.DISABLED;
    resetCommandsAndRoutines();

    if (!setup) {
      tab.add(autoChooser);

      autoChooser.setDefaultOption("None", new None());
      //      autoChooser.addOption("Test", new TestAuto());
      autoChooser.addOption("Preload Move Out", new Preload());
      //      autoChooser.addOption("L + 2", new LPlus2());
      autoChooser.addOption("CleanSideCenterRush", new CleanSideCenterRush());
      //      autoChooser.addOption("Initial3Piece", new Initial3PieceAuto());
      autoChooser.addOption("ComeAndGo", new ComeAndGo());
      autoChooser.addOption("OneThreeComeAndGo", new OneThreeComeAndGo());
      autoChooser.addOption("TwoThreeComeAndGo", new TwoThreeComeAndGo());
      //      autoChooser.addOption("TwoFourThree", new MidAuto());
      autoChooser.addOption("SevenPieceCenterRush", new SevenPieceCenterRush());
      autoChooser.addOption("CleanSideFourThree", new CleanSideFourThree());
      //      autoChooser.addOption("Test", new SevenPreloadTest());

      setup = true;
    }
  }

  @Override
  public void autonomousInit() {
    startStage(RobotState.GamePeriod.AUTO);

    RobotController.setBrownoutVoltage(5.75);

    HardwareAdapter.SwerveHardware.getInstance().gyro.setYaw(0);
    // Auto routine is being set in disabled periodic
    mCommands.addWantedRoutine(autoRoutine);
    Logger.getInstance().log("Selected Auto", autoRoutine.getName());
    autoTimer.restart();
  }

  private void startStage(RobotState.GamePeriod period) {
    mRobotState.gamePeriod = period;
    resetCommandsAndRoutines();
    readRobotState();
    Swerve.getInstance().setOutputFromMeasured(mRobotState.realModuleStates);
  }

  @Override
  public void teleopInit() {
    startStage(RobotState.GamePeriod.TELEOP);

    RobotController.setBrownoutVoltage(5.75);
  }

  @Override
  public void testInit() {
    startStage(RobotState.GamePeriod.TESTING);
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().log("GameState", mRobotState.gamePeriod.name());
    SmartDashboard.putString("GameState", mRobotState.gamePeriod.name());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  @Override
  public void simulationPeriodic() {
    mRobotState.gyroHeading =
        mRobotState.gyroHeading.plus(
            new Rotation2d(mRobotState.chassisRelativeSpeeds.omegaRadiansPerSecond * 0.02));
  }

  DriverStation.Alliance gotten = DriverStation.Alliance.Blue;
  PowerDistribution distribution = new PowerDistribution();

  @Override
  public void disabledPeriodic() {
    sLoopDebugger.reset();

    if (distribution.getVoltage() < 12.5) {
      HardwareAdapter.LightingHardware.getInstance().candle.animate(LightingConstants.lowBattery);
    } else {
      HardwareAdapter.LightingHardware.getInstance().candle.animate(LightingConstants.rainbow);
    }

    Optional<DriverStation.Alliance> optionalAlliance = DriverStation.getAlliance();
    optionalAlliance.ifPresent(alliance -> gotten = alliance);

    onBlueAlliance = gotten == DriverStation.Alliance.Blue;
    SmartDashboard.putBoolean("onBlueAlliance", onBlueAlliance);

    readRobotState();

    autoRoutine = autoChooser.getSelected().getRoutine();

    sLoopDebugger.finish();
  }

  @Override
  public void autonomousPeriodic() {
    sLoopDebugger.reset();
    readRobotState();
    mRoutineManager.update(mCommands, mRobotState);

    SmartDashboard.putNumber("Auto/Time", Math.round(autoTimer.get() * 100) / 100.0);
    SmartDashboard.putString("Auto/Wanted/Shooter", "shooter-" + mCommands.shooterWanted.name());
    SmartDashboard.putString("Auto/Wanted/Kicker", "kicker-" + mCommands.kickerWanted.name());
    SmartDashboard.putString("Auto/Wanted/Arm", "arm-" + mCommands.wantedArm.name());
    SmartDashboard.putString("Auto/Wanted/Turret", "turret-" + mCommands.turretWanted.name());
    SmartDashboard.putString("Auto/Wanted/Intake", "intake-" + mCommands.intakeWanted.name());

    Logger.getInstance().log("Auto/Wanted/Shooter", "shooter-" + mCommands.shooterWanted.name());
    Logger.getInstance().log("Auto/Wanted/Kicker", "kicker-" + mCommands.kickerWanted.name());
    Logger.getInstance().log("Auto/Wanted/Arm", "arm-" + mCommands.wantedArm.name());
    Logger.getInstance().log("Auto/Wanted/Turret", "turret-" + mCommands.turretWanted.name());
    Logger.getInstance().log("Auto/Wanted/Intake", "intake-" + mCommands.intakeWanted.name());

    updateSubsystemsAndApplyOutputs();
    sLoopDebugger.finish();
  }

  public static double lastDt = 0;

  @Override
  public void teleopPeriodic() {
    sLoopDebugger.reset();
    readRobotState();
    SmartDashboard.putString("Auto/Wanted/Shooter", "shooter-" + mCommands.shooterWanted.name());
    SmartDashboard.putString("Auto/Wanted/Kicker", "kicker-" + mCommands.kickerWanted.name());
    SmartDashboard.putString("Auto/Wanted/Arm", "arm-" + mCommands.wantedArm.name());
    SmartDashboard.putString("Auto/Wanted/Turret", "turret-" + mCommands.turretWanted.name());
    SmartDashboard.putString("Auto/Wanted/Intake", "intake-" + mCommands.intakeWanted.name());
    try {
      mOperatorInterface.updateCommands(mCommands, mRobotState);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
    mRoutineManager.update(mCommands, mRobotState);
    updateSubsystemsAndApplyOutputs();
    sLoopDebugger.finish();
    lastDt = sLoopDebugger.mTimer.get();
  }

  @Override
  public void testPeriodic() {
    teleopPeriodic();
  }

  private void resetCommandsAndRoutines() {
    mOperatorInterface.reset(mCommands);
    mRoutineManager.clearRunningRoutines();
    updateSubsystemsAndApplyOutputs();
  }

  private void readRobotState() {
    mHardwareReader.readState(mSubsystems, mRobotState);
  }

  private void resetOdometryIfWanted() {
    Pose2d wantedPose = mCommands.driveWantedOdometryPose;
    Rotation2d wantedPoseRotation = mCommands.driveWantedOdometryPoseRotation;
    if (wantedPose != null && wantedPoseRotation != null) {
      mRobotState.poseEst.resetPosition(
          wantedPoseRotation,
          new SwerveModulePosition[] {
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0))
          },
          wantedPose);
      System.out.println("RESET POSE");
      mCommands.driveWantedOdometryPose = null;
      mCommands.driveWantedOdometryPoseRotation = null;
    }
  }

  private void updateSubsystemsAndApplyOutputs() {
    for (SubsystemBase subsystem : mSubsystems) {
      subsystem.update(mCommands, mRobotState);
      sLoopDebugger.addPoint("Update " + subsystem.getName());
    }
    mHardwareWriter.writeHardware(mSubsystems, mRobotState);
    sLoopDebugger.addPoint("updateSubsystemsAndApplyOutputs");
  }

  public class OdometryThread extends Thread {

    private final HardwareAdapter.SwerveHardware mSwerveHardwareInstance;
    private final HardwareAdapter.VisionHardware mVisionHardwareInstance;
    private final BaseStatusSignal[] mAllSignals;
    private final RobotState mRobotState;
    public boolean mIsRunning = true;

    public StatusSignal<Double> yaw;
    public StatusSignal<Double> yawRate;

    public OdometryThread(RobotState state) {
      super();

      mRobotState = state;
      mSwerveHardwareInstance = HardwareAdapter.SwerveHardware.getInstance();
      mVisionHardwareInstance = HardwareAdapter.VisionHardware.getInstance();

      mAllSignals = new BaseStatusSignal[2 + 4 * 4]; // 2 gyro + 4 * 4 for module motors
      int modCount = 0;
      for (SwerveModule module : mSwerveHardwareInstance.modules) {
        BaseStatusSignal[] moduleSignals = module.getBaseSignals();
        for (int i = 0; i < 4; i++) {
          mAllSignals[(modCount * 4) + i] = moduleSignals[i];
        }
        modCount++;
      }

      mAllSignals[mAllSignals.length - 2] = yaw = mSwerveHardwareInstance.gyro.getYaw();
      mAllSignals[mAllSignals.length - 1] =
          yawRate = mSwerveHardwareInstance.gyro.getAngularVelocityZWorld();
    }

    public void run() {

      BaseStatusSignal.setUpdateFrequencyForAll(250, mAllSignals);
      Threads.setCurrentThreadPriority(true, 1);

      while (mIsRunning) {
        long loopStartNano = System.nanoTime();
        // Read swerve hardware values (motors + gyro)
        BaseStatusSignal.waitForAll(2.0 / 250.0, mAllSignals);

        if (Robot.isRobotReal()) {
          mRobotState.gyroHeading =
              Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValue(yaw, yawRate));
        }

        if (!Robot.isRobotReal()) {
          mRobotState.realModulePositions = Swerve.getInstance().getOutputs().getPositions();
          mRobotState.realModuleStates = Swerve.getInstance().getOutputs().getStates();
        } else {
          for (int i = 0; i < 4; i++) {
            SwerveModuleState a = mSwerveHardwareInstance.modules[i].getState();
            mRobotState.realModuleStates[i] = a;
          }
          mRobotState.realModulePositions =
              new SwerveModulePosition[] {
                mSwerveHardwareInstance.modules[0].getPosition(false),
                mSwerveHardwareInstance.modules[1].getPosition(false),
                mSwerveHardwareInstance.modules[2].getPosition(false),
                mSwerveHardwareInstance.modules[3].getPosition(false)
              };
        }

        // Update perception results
        mRobotState.perceptionResults.clear();
        for (PerceptionCamera camera : mVisionHardwareInstance.cameras) {
          mRobotState.perceptionResults.addAll(camera.getLatestResults());
        }
        mVision.update(mCommands, mRobotState);

        // Update odometry pose estimation using values from above
        RobotStateEstimator.getInstance().updateSwervePoseEstimator(mRobotState);
        mRobotState.lastEstAngle = mRobotState.poseEst.getEstimatedPosition().getRotation();
        mRobotState.lastEst = mRobotState.poseEst.getEstimatedPosition().times(1);

        resetOdometryIfWanted();

        long loopEndNano = System.nanoTime();

        SmartDashboard.putNumber("Odometry loop time", (loopEndNano - loopStartNano) / 1_000_000.0);
      }
    }
  }
}
