package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.constants.Alliance;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utils.shooterMath.ShooterMath;
import java.util.function.Supplier;

@Logged
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
  @NotLogged private static final double kSimLoopPeriod = 0.005; // 5 ms

  @NotLogged private Notifier m_simNotifier = null;

  @Logged(importance = Importance.DEBUG)
  private double m_lastSimTime;

  @NotLogged
  private final FieldCentric m_fieldCentric =
      new FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

  @NotLogged
  public final FieldCentricFacingAngle m_fieldCentricFacingAngle =
      new FieldCentricFacingAngle()
          .withHeadingPID(6.0, 0, 0)
          .withDriveRequestType(DriveRequestType.Velocity);

  @NotLogged
  public final RobotCentric m_robotCentricBLine =
      new RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

  @NotLogged private DriveStates m_currentState = DriveStates.DriverControlled;

  @NotLogged private boolean m_sysid = false;

  /** Controller inputs for default teleop */
  @NotLogged private CommandXboxController m_inputController;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  @NotLogged private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;

  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  @NotLogged private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

  /* Keep track if we've ever applied the operator perspective before or not */
  @NotLogged private boolean m_hasAppliedOperatorPerspective = false;

  /* Override default swerve request for a higher priority one (used in auto) */
  @NotLogged private boolean m_autonomousRequestOverride = false;

  @NotLogged private boolean m_shouldAcceptNextVisionMeasurementRotation = false;

  @NotLogged private SwerveDriveState m_cachedState;

  @Logged(importance = Importance.CRITICAL)
  private boolean m_brownoutMode = false;

  private LinearVelocity m_currentVelocity = TunerConstants.kSpeedAt12Volts;

  // SysId routines

  ///////////////////// Steer ///////////////////////

  // private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
  //     new SwerveRequest.SysIdSwerveSteerGains();

  // private final SysIdRoutine m_sysIdRoutineToApply =
  //     new SysIdRoutine(
  //         new SysIdRoutine.Config(
  //             /*
  //              * This is in radians per second squared, but SysId only supports
  //              * "volts per second"
  //              */
  //             Volts.of(1).per(Second),
  //             /* This is in radians per second, but SysId only supports "volts" */
  //             Volts.of(6),
  //             Seconds.of(9),
  //             // Log state with Logger class
  // state -> SignalLogger.writeString("SysId_State", state.toString())),
  //         new SysIdRoutine.Mechanism(
  //             output -> {
  //               setControl(m_steerCharacterization.withVolts(output.in(Volts)));
  //               Robot.telemetry().log("Steer_Rate", output.in(Volts));
  //             },
  //             null,
  //             this));

  //////////////////////////////// Translation /////////////////////////

  // @NotLogged
  // private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
  //     new SwerveRequest.SysIdSwerveTranslation();

  // @NotLogged
  // private final SysIdRoutine m_sysIdRoutineToApply =
  //     new SysIdRoutine(
  //         new SysIdRoutine.Config(
  //             Volts.of(1).per(Second),
  //             Volts.of(7),
  //             null, // Use default timeout (10 s)
  //             // Log state with Logger class
  //             state -> SignalLogger.writeString("SysId_State", state.toString())),
  //         new SysIdRoutine.Mechanism(
  //             output -> {
  //               setControl(m_translationCharacterization.withVolts(output.in(Volts)));
  //               Robot.telemetry().log("Translation_Rate", output.in(Volts));
  //             },
  //             null,
  //             this));

  /////////////////////////// Rotation /////////////////////////
  ///
  @NotLogged
  public final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  @NotLogged
  private final SysIdRoutine m_sysIdRoutineToApply =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /*
               * This is in radians per second squared, but SysId only supports
               * "volts per second"
               */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with Logger class
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                Robot.telemetry().log("Rotation_Rate", output.in(Volts));
              },
              null,
              this));

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    // Epilogue doesn't like null values and new SwerveDriveState() has a few null values :(
    m_cachedState = getState();
    for (SwerveModule<TalonFX, TalonFX, CANcoder> module : getModules()) {
      module.getDriveMotor().getConfigurator().apply(TunerConstants.kNormalCurrentLimits, 0.0);
    }
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    // Epilogue doesn't like null values and new SwerveDriveState() has a few null values :(
    m_cachedState = getState();
    for (SwerveModule<TalonFX, TalonFX, CANcoder> module : getModules()) {
      module.getDriveMotor().getConfigurator().apply(TunerConstants.kNormalCurrentLimits, 0.0);
    }
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    // Epilogue doesn't like null values and new SwerveDriveState() has a few null values :(
    m_cachedState = getState();
    for (SwerveModule<TalonFX, TalonFX, CANcoder> module : getModules()) {
      module.getDriveMotor().getConfigurator().apply(TunerConstants.kNormalCurrentLimits, 0.0);
    }
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return Commands.run(() -> applyRequest(requestSupplier.get()));
  }

  /** Applies the given swerve request, respecting autonomous override. */
  public void applyRequest(SwerveRequest request) {
    if (!DriverStation.isAutonomous() || !m_autonomousRequestOverride) {
      setControl(request);
    }
  }

  /** Applies a high-priority swerve request during autonomous. */
  public void applyPriorityRequestAuto(SwerveRequest request) {
    if (DriverStation.isAutonomous() && m_autonomousRequestOverride) {
      setControl(request);
    }
  }

  /** Runs periodic drivetrain logic including operator perspective and drive control. */
  public void periodic() {
    // Prevents many calls to getState() because it's a blocking operation
    m_cachedState = getState();

    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      setOperatorPerspectiveForward(
          Alliance.redAlliance
              ? kRedAlliancePerspectiveRotation
              : kBlueAlliancePerspectiveRotation);

      m_hasAppliedOperatorPerspective = true;
    }

    ShooterMath.calculate(getPose());
  }

  private void runState() {
    if (!DriverStation.isAutonomous() && !m_sysid && DriverStation.isEnabled()) {
      Vector<N2> scaledTranslationInputs =
          rescaleTranslation(m_inputController.getLeftY(), m_inputController.getLeftX());
      switch (m_currentState) {
        case DriverControlled:
          setControl(
              m_fieldCentric
                  .withVelocityX(m_currentVelocity.times(-scaledTranslationInputs.get(0, 0)))
                  .withVelocityY(m_currentVelocity.times(-scaledTranslationInputs.get(1, 0)))
                  .withRotationalRate(
                      TunerConstants.kMaxAngularRate.times(
                          -rescaleRotation(m_inputController.getRightX()))));
          break;
        case RotationLock:
          setControl(
              m_fieldCentricFacingAngle
                  .withVelocityX(m_currentVelocity.times(-scaledTranslationInputs.get(0, 0)))
                  .withVelocityY(m_currentVelocity.times(-scaledTranslationInputs.get(1, 0)))
                  .withTargetRateFeedforward(
                      TunerConstants.kMaxAngularRate.times(
                          -rescaleRotation(m_inputController.getRightX()))));
          break;
      }
    }
  }

  public void sysid(boolean sysid) {
    m_sysid = sysid;
  }

  /** Rescales the translation input vector with deadband and power curve. */
  public Vector<N2> rescaleTranslation(double x, double y) {
    return MathUtil.copyDirectionPow(MathUtil.applyDeadband(VecBuilder.fill(x, y), 0.075), 2);
  }

  /** Rescales the rotation input with deadband. */
  public double rescaleRotation(double rotation) {
    return Math.copySign(MathUtil.applyDeadband(Math.abs(rotation), 0.075), rotation);
  }

  /** Sets whether autonomous requests should override normal control. */
  public void setAutonomousRequestOverride(boolean override) {
    m_autonomousRequestOverride = override;
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to {@link
   * #setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement in the form
   *     [x, y, theta]ᵀ, with units in meters and radians.
   */
  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    if (m_shouldAcceptNextVisionMeasurementRotation) {
      m_shouldAcceptNextVisionMeasurementRotation = false;
      resetRotation(visionRobotPoseMeters.getRotation());
    }
  }

  public void setShouldAcceptNextVisionMeasurementRotation(boolean shouldAccept) {
    m_shouldAcceptNextVisionMeasurementRotation = shouldAccept;
  }

  public void toggleBrownout() {
    m_brownoutMode = !m_brownoutMode;
    Robot.telemetry().log("Brownout/Drivetrain", m_brownoutMode);
    for (SwerveModule<TalonFX, TalonFX, CANcoder> module : getModules()) {
      module
          .getDriveMotor()
          .getConfigurator()
          .apply(
              m_brownoutMode
                  ? TunerConstants.kBrownoutCurrentLimits
                  : TunerConstants.kNormalCurrentLimits,
              0.0);
    }
    if (m_brownoutMode) {
      m_currentVelocity = TunerConstants.kSpeedBrownout;
    } else {
      m_currentVelocity = TunerConstants.kSpeedAt12Volts;
    }
  }

  @Logged(importance = Importance.CRITICAL)
  public Pose2d getPose() {
    return m_cachedState.Pose;
  }

  @NotLogged
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  @Logged(importance = Importance.CRITICAL)
  public SwerveModuleState[] getModuleStates() {
    return m_cachedState.ModuleStates;
  }

  @Logged(importance = Importance.CRITICAL)
  public SwerveModuleState[] getModuleTargets() {
    return m_cachedState.ModuleTargets;
  }

  @NotLogged
  public ChassisSpeeds getRobotSpeeds() {
    return m_cachedState.Speeds;
  }

  @Logged(importance = Importance.CRITICAL)
  public ChassisSpeeds getFieldSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotSpeeds(), getRotation());
  }

  @Logged(importance = Importance.CRITICAL)
  public ChassisSpeeds getTargetFieldSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        getKinematics().toChassisSpeeds(getModuleTargets()), getRotation());
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  /** Set the {@link DriveStates#DriverControlled} and assists controller */
  public void setController(CommandXboxController controller) {
    m_inputController = controller;
  }

  /** Set the {@link DriveStates#RotationLock} target */
  public void setRotationTarget(Rotation2d target) {
    m_fieldCentricFacingAngle.TargetDirection = target;
  }

  public Rotation2d getRotationTarget() {
    return m_fieldCentricFacingAngle.TargetDirection;
  }

  /** Sets the current drive state. */
  public void setState(DriveStates state) {
    m_currentState = state;
    runState();
  }

  public enum DriveStates {
    DriverControlled,
    RotationLock,
  }
}
