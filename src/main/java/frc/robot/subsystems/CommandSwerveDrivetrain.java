package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utils.CustomFieldCentric;
import frc.robot.utils.Log;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
@Logged
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private AngularVelocity MaxAngularRate = RotationsPerSecond.of(2);

  private final CustomFieldCentric fieldCentric = new CustomFieldCentric();

  private DriveStates currentState = DriveStates.DRIVER_CONTROLLED;

  /** Controller inputs for default teleop */
  private CommandXboxController inputController;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

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
  //             state -> Log.log("SysIdSteer_State", state.toString())),
  //         new SysIdRoutine.Mechanism(
  //             output -> {
  //               setControl(m_steerCharacterization.withVolts(output.in(Volts)));
  //               Log.log("Steer_Rate", output.in(Volts));
  //             },
  //             null,
  //             this));

  //////////////////////////////// Translation /////////////////////////

  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();

  private final SysIdRoutine m_sysIdRoutineToApply =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(1).per(Second),
              Volts.of(7),
              null, // Use default timeout (10 s)
              // Log state with Logger class
              state -> Log.log("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                setControl(m_translationCharacterization.withVolts(output.in(Volts)));
                Log.log("Translation_Rate", output.in(Volts));
              },
              null,
              this));

  /////////////////////////// Rotation /////////////////////////

  // private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
  //     new SwerveRequest.SysIdSwerveRotation();

  // private final SysIdRoutine m_sysIdRoutineToApply =
  //     new SysIdRoutine(
  //         new SysIdRoutine.Config(
  //             /*
  //              * This is in radians per second squared, but SysId only supports
  //              * "volts per second"
  //              */
  //             Volts.of(Math.PI / 6).per(Second),
  //             /* This is in radians per second, but SysId only supports "volts" */
  //             Volts.of(Math.PI),
  //             null, // Use default timeout (10 s)
  //             // Log state with Logger class
  //             state -> Log.log("SysIdSteer_State", state.toString())),
  //         new SysIdRoutine.Mechanism(
  //             output -> {
  //               setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
  //               Log.log("Rotation_Rate", output.in(Volts));
  //             },
  //             null,
  //             this));

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians
   * @param modules Constants for each specific module
   */
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
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return Commands.run(() -> this.setControl(requestSupplier.get()));
  }

  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      setOperatorPerspectiveForward(
          Alliance.redAlliance
              ? kRedAlliancePerspectiveRotation
              : kBlueAlliancePerspectiveRotation);
      m_hasAppliedOperatorPerspective = true;
    }

    Vector<N2> scaledTranslationInputs =
        rescaleTranslation(inputController.getLeftY(), inputController.getLeftX());

    setControl(
        fieldCentric
            .withVelocityX(MaxSpeed.times(-scaledTranslationInputs.get(0, 0)))
            .withVelocityY(MaxSpeed.times(-scaledTranslationInputs.get(1, 0)))
            .withRotationalRate(MaxAngularRate.times(-rescaleRotation(inputController.getRightX())))
            .withDriveState(currentState));
  }

  public Vector<N2> rescaleTranslation(double x, double y) {
    Vector<N2> scaledJoyStick = VecBuilder.fill(x, y);
    scaledJoyStick = MathUtil.applyDeadband(scaledJoyStick, 0.075);
    return MathUtil.copyDirectionPow(scaledJoyStick, 2);
  }

  public double rescaleRotation(double rotation) {
    return Math.copySign(MathUtil.applyDeadband(rotation, 0.075), rotation);
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
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
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
  }

  public Pose2d getPose() {
    return getState().Pose;
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public SwerveModuleState[] getModuleStates() {
    return getState().ModuleStates;
  }

  public SwerveModuleState[] getModuleTargets() {
    return getState().ModuleTargets;
  }

  public ChassisSpeeds getRobotSpeeds() {
    return getState().Speeds;
  }

  public ChassisSpeeds getFieldSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotSpeeds(), getRotation());
  }

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

  /** Returns true if the robot is in its own alliance zone. */
  public boolean inAllianceZone() {
    return (Alliance.redAlliance
        ? getPose().getX()
            > FieldConstants.kFieldLength.minus(FieldConstants.kStartingLineDistance).in(Meters)
        : getPose().getX() < FieldConstants.kFieldLength.in(Meters));
  }

  /** Set the {@link DriveStates#DRIVER_CONTROLLED} and assists controller */
  public void setController(CommandXboxController controller) {
    this.inputController = controller;
  }

  /**
   * Set the {@link DriveStates#Y_ASSIST} target relative to the center of the field (not side
   * relative)
   */
  public void setYTargetFromCenter(Distance target) {
    fieldCentric.withYTargetFromCenter(target);
  }

  /** Set the {@link DriveStates#ROTATION_LOCK} target */
  public void setRotationTarget(Rotation2d target) {
    fieldCentric.withTargetRotation(target);
  }

  public void setState(DriveStates state) {
    this.currentState = state;
  }

  public enum DriveStates {
    DRIVER_CONTROLLED,
    Y_ASSIST,
    ROTATION_LOCK,
  }
}
