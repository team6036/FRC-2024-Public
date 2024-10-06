package com.peninsula.frc2024.robot;

import com.peninsula.frc2024.config.SwerveConstants;
import com.peninsula.frc2024.subsystems.Lighting;
import com.peninsula.frc2024.subsystems.Limelight;
import com.peninsula.frc2024.subsystems.Shooter;
import com.peninsula.frc2024.subsystems.Vision;
import com.peninsula.frc2024.util.field3d.Field3d;
import com.peninsula.frc2024.util.peninsulaCoolios.phronesis.fieldAwareness.FieldObserver;
import com.peninsula.frc2024.util.peninsulaCoolios.phronesis.fieldAwareness.FieldState;
import com.peninsula.frc2024.util.peninsulaCoolios.trajectories.WantedTrajectory;
import com.peninsula.frc2024.vision.LimelightHelpers;
import com.peninsula.frc2024.vision.PerceptionObservation;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.ArrayList;
import java.util.Set;

/** Holds the current physical state of the robot from our sensors. */
@SuppressWarnings("java:S1104")
public class RobotState {

  public enum GamePeriod {
    AUTO,
    TELEOP,
    TESTING,
    DISABLED
  }

  /* Swerve */
  public Rotation2d gyroHeading = new Rotation2d();
  //  public double gyroPitch = 0;
  //  public double gyroRoll = 0;
  public double[] moduleEncoderPos = new double[4];

  public SwerveDrivePoseEstimator poseEst =
      new SwerveDrivePoseEstimator(
          SwerveConstants.kKinematics,
          new Rotation2d(0),
          new SwerveModulePosition[] {
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0)),
          },
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, 0.05),
          VecBuilder.fill(3.0, 3.0, 10.0));

  public SwerveModulePosition[] realModulePositions;
  public SwerveModuleState[] realModuleStates = new SwerveModuleState[4];

  {
    for (int i = 0; i < 4; i++) {
      realModuleStates[i] = new SwerveModuleState();
    }
  }

  public double odometryDeadReckonUpdateTime = 0;
  public Field2d m_field = new Field2d();
  public Field3d m_field3d = new Field3d();
  public ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(0, 0, 0);
  public ChassisSpeeds chassisRelativeSpeeds = new ChassisSpeeds(0, 0, 0);

  public TimeInterpolatableBuffer<Pose2d> pastPoses = TimeInterpolatableBuffer.createBuffer(1.0);

  /* Joystick */
  public double driverLeftX, driverRightX, driverLeftY, driverRt = 0;
  public double driverRightY;
  public boolean operatorAPressed,
      operatorB,
      operatorBPressed,
      operatorXPressed,
      operatorYPressed,
      operatorRtPressed,
      operatorLtPressed,
      operatorRbPressed,
      operatorLbPressed,
      operatorDPadLeftPressed,
      operatorDPadRightPressed;
  public double operatorLeftX, operatorLeftY, operatorRightY, operatorRightX;
  public boolean driverAPressed, driverBPressed, driverXPressed, driverYPressed;
  public boolean driverLtPressed;
  public boolean driverLbPressed;
  public boolean driverRbPressed;
  public boolean driverDPadUp, driverDPadDown, driverDPadLeft, driverDPadRight;
  public boolean driverFSDLock;

  public boolean lastDPadPressed = false;
  public boolean operatorDPadUp;
  public boolean operatorDPadDown;
  public boolean operatorDPadRight;
  public boolean operatorDPadLeft;
  public boolean operatorLtLastPressed = false;
  public boolean operatorYLastPressed = false;

  public double rumbleDriver = 0;
  public double rumbleOperator = 0;

  /* Vision */
  public Vision.State visionWanted = Vision.State.ON;
  public ArrayList<PerceptionObservation> perceptionResults = new ArrayList<>();
  public Set<Integer> ignoredTags = Set.of();

  /* Miscellaneous */
  public GamePeriod gamePeriod = GamePeriod.DISABLED;
  public String gameData;
  public double gameTimeS = 0;

  /* Auto */
  public WantedTrajectory currentTrajectory;
  public Pose2d initPose = new Pose2d(0, 0, new Rotation2d(0));

  /* Lighting */
  public Lighting.State lastSet = Lighting.State.OFF;

  /* FSD */
  public boolean runningFSD = false;

  /* Arm */
  public double armPos = 0;
  public double armPoseRef = 0;

  public boolean climbMode = false;
  public boolean climbUp = false;

  /* Turret */
  public double turretMotorPosition = 0;
  public double turretMotorPositionRef = 0;
  public double turretAbsoluteEncoderPosition = 0;

  public Rotation2d lastEstAngle = new Rotation2d();
  public Pose2d lastEst = new Pose2d();

  /* SOTM */
  public Pose2d virtual_goal = new Pose2d();
  public Pose2d hoard_virtual_goal = new Pose2d();
  public double dist_to_target = 0;

  /* Shooter */
  public boolean pieceInKick = true;
  public boolean backSensor = false;
  public double backSensorOnTime = 0.0;
  public double pieceLeftTime = -10;
  public double rightV = 0, leftV = 0;
  public double rightVRef = 0, leftVRef = 0;

  public double timeOfLastFallingEdge;
  public double timeOfLastRisingEdge;

  public Shooter.SourceIntakeStage sourceIntakeStage = Shooter.SourceIntakeStage.INTAKE;

  public double shooterTrimBlue = 0.0025;
  public double shooterTrimRed = 0.0025;
  public double shooterTrim = Robot.onBlueAlliance ? shooterTrimBlue : shooterTrimRed;

  /* Field */
  public FieldState state = new FieldState();
  public FieldObserver fieldObserver = new FieldObserver();

  public LimelightHelpers.LimelightResults limelightResults;
  public Limelight.State limelightWanted = Limelight.State.ON;
}
