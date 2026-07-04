package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
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
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import frc.robot.Robot;
import frc.robot.constants.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utils.CustomFieldCentric;
import frc.robot.utils.shooterMath.ShooterMath4;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms

  private Notifier m_simNotifier = null;

  private double m_lastSimTime;

  private LinearVelocity m_maxSpeed = TunerConstants.kSpeedAt12Volts;

  private AngularVelocity m_maxAngularRate = TunerConstants.kMaxAngularRate;

  public final CustomFieldCentric fieldCentric;

  public final RobotCentric fieldCentricBLine;

  private DriveStates m_currentState = DriveStates.DRIVER_CONTROLLED;

  private boolean m_sysid = false;

  /** Controller inputs for default teleop */
  private CommandXboxController m_inputController;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;

  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /* Override default swerve request for a higher priority one (used in auto) */
  private boolean m_autonomousRequestOverride = false;

  private boolean m_shouldAcceptNextVisionMeasurementRotation = false;

  @SuppressWarnings("unused")
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();

  @SuppressWarnings("unused")
  private final SysIdRoutine m_sysIdRoutineToApply =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(1).per(Second),
              Volts.of(7),
              null,
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                setControl(m_translationCharacterization.withVolts(output.in(Volts)));
                Logger.recordOutput("Swerve/Translation_Rate", output.in(Volts));
              },
              null,
              this));

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    fieldCentric = new CustomFieldCentric(getPigeon2());
    fieldCentricBLine = new RobotCentric();
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    fieldCentric = new CustomFieldCentric(getPigeon2());
    fieldCentricBLine = new RobotCentric();
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
    fieldCentric = new CustomFieldCentric(getPigeon2());
    fieldCentricBLine = new RobotCentric();
  }

  public void setTeleCurrentLimits() {
    for (int i = 0; i < getModules().length; i++) {
      getModules()[i]
          .getDriveMotor()
          .getConfigurator()
          .apply(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(110)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(45)
                  .withSupplyCurrentLimitEnable(true));
    }
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return Commands.run(() -> this.applyRequest(requestSupplier.get()));
  }

  public void applyRequest(SwerveRequest request) {
    if (!DriverStation.isAutonomous() || !m_autonomousRequestOverride) {
      this.setControl(request);
    }
  }

  public void applyPriorityRequestAuto(SwerveRequest request) {
    if (DriverStation.isAutonomous() && m_autonomousRequestOverride) {
      this.setControl(request);
    }
  }

  public void periodic() {
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      setOperatorPerspectiveForward(
          Alliance.redAlliance
              ? kRedAlliancePerspectiveRotation
              : kBlueAlliancePerspectiveRotation);
      m_hasAppliedOperatorPerspective = true;
    }

    Vector<N2> scaledTranslationInputs =
        rescaleTranslation(m_inputController.getLeftY(), m_inputController.getLeftX());

    if (!DriverStation.isAutonomous() && !m_sysid) {
      setControl(
          fieldCentric
              .withVelocityX(m_maxSpeed.times(-scaledTranslationInputs.get(0, 0)))
              .withVelocityY(m_maxSpeed.times(-scaledTranslationInputs.get(1, 0)))
              .withRotationalRate(
                  m_maxAngularRate.times(-rescaleRotation(m_inputController.getRightX())))
              .withDriveState(m_currentState));
    }

    ShooterMath4.calculate(getPose());

    // Log drivetrain state periodically
    Logger.recordOutput("Swerve/CurrentState", m_currentState.name());
    Logger.recordOutput("Swerve/LastSimTime", m_lastSimTime);
    Logger.recordOutput("Swerve/AutonomousRequestOverride", m_autonomousRequestOverride);
    Logger.recordOutput("Swerve/Pose", getPose(), Pose2d.struct);
    Logger.recordOutput("Swerve/Rotation", getRotation().getDegrees());
    Logger.recordOutput("Swerve/ModuleStates", getModuleStates());
    Logger.recordOutput("Swerve/ModuleTargets", getModuleTargets());
    Logger.recordOutput("Swerve/RobotSpeeds", getRobotSpeeds(), ChassisSpeeds.struct);
    Logger.recordOutput("Swerve/FieldSpeeds", getFieldSpeeds(), ChassisSpeeds.struct);
    Logger.recordOutput("Swerve/TargetFieldSpeeds", getTargetFieldSpeeds(), ChassisSpeeds.struct);
    Logger.recordOutput("Swerve/InAllianceZone", inAllianceZone());
  }

  public void sysid(boolean sysid) {
    m_sysid = sysid;
  }

  public Vector<N2> rescaleTranslation(double x, double y) {
    return MathUtil.copyDirectionPow(MathUtil.applyDeadband(VecBuilder.fill(x, y), 0.075), 2);
  }

  public double rescaleRotation(double rotation) {
    return Math.copySign(MathUtil.applyDeadband(Math.abs(rotation), 0.075), rotation);
  }

  public void setAutonomousRequestOverride(boolean override) {
    this.m_autonomousRequestOverride = override;
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    if (m_shouldAcceptNextVisionMeasurementRotation) {
      m_shouldAcceptNextVisionMeasurementRotation = false;
      resetRotation(visionRobotPoseMeters.getRotation().plus(Rotation2d.k180deg));
    }
  }

  public void setShouldAcceptNextVisionMeasurementRotation(boolean shouldAccept) {
    this.m_shouldAcceptNextVisionMeasurementRotation = shouldAccept;
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

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  public boolean inAllianceZone() {
    return (Alliance.redAlliance
        ? getPose().getX()
            > FieldConstants.kFieldLength.minus(FieldConstants.kStartingLineDistance).in(Meters)
        : getPose().getX() < FieldConstants.kFieldLength.in(Meters));
  }

  public void setController(CommandXboxController controller) {
    this.m_inputController = controller;
  }

  public void setRotationTarget(Rotation2d target) {
    fieldCentric.withTargetRotation(target);
  }

  public void setState(DriveStates state) {
    this.m_currentState = state;
  }

  public enum DriveStates {
    DRIVER_CONTROLLED,
    ROTATION_LOCK,
  }
}