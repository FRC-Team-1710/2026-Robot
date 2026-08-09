package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.constants.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.MatchState;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveStates;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.Feeder.FeederStates;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterStates;
import frc.robot.utils.shooterMath.ShooterMath;

@Logged
public class Superstructure {
  @NotLogged private CommandXboxController m_driver;
  @NotLogged private CommandSwerveDrivetrain m_drivetrain;
  @NotLogged private Intake m_intake;
  @NotLogged private Shooter m_shooter;
  @NotLogged private Indexer m_indexer;
  @NotLogged private Feeder m_feeder;

  @Logged(importance = Importance.CRITICAL)
  private WantedStates m_wantedState = WantedStates.Default;

  @Logged(importance = Importance.CRITICAL)
  private CurrentStates m_currentState = CurrentStates.Idle;

  @Logged(importance = Importance.CRITICAL)
  private IntakeAddableStates m_intakeAddableState = IntakeAddableStates.Intaking;

  @Logged(importance = Importance.CRITICAL)
  private ShooterSubStates m_shooterSubState = ShooterSubStates.Score;

  @NotLogged private final Debouncer m_readyToShootDebouncer = new Debouncer(0.1);

  @NotLogged private final Debouncer m_driveAtTargetDebouncer = new Debouncer(0.1);

  @Logged(importance = Importance.CRITICAL)
  private boolean m_wasAtTarget = false;

  @NotLogged private final Timer m_timer = new Timer();

  @NotLogged private double m_restingVoltage = -1;

  /**
   * Constructs the superstructure with all subsystem references.
   *
   * @param driver the driver controller
   * @param drivetrain the swerve m_drivetrain
   * @param intake the m_intake subsystem
   * @param shooter the m_shooter subsystem
   * @param indexer the m_indexer subsystem
   * @param feeder the m_feeder subsystem
   */
  public Superstructure(
      CommandXboxController driver,
      CommandSwerveDrivetrain drivetrain,
      Intake intake,
      Shooter shooter,
      Indexer indexer,
      Feeder feeder) {
    m_driver = driver;
    m_drivetrain = drivetrain;
    m_intake = intake;
    m_shooter = shooter;
    m_indexer = indexer;
    m_feeder = feeder;
  }

  /** Runs periodic logic for state transitions and subsystem coordination. */
  public void periodic() {
    m_currentState = handleStateTransitions();
    applyStates();

    applyRumble();

    Robot.telemetry()
        .log(
            "MatchState/TimeTillActive",
            Math.round(MatchState.timeTillActive().in(Seconds) * 10.0) / 10.0);
    Robot.telemetry()
        .log(
            "MatchState/TimeTillInactive",
            Math.round(MatchState.timeTillInactive().in(Seconds) * 10.0) / 10.0);
    Robot.telemetry()
        .log(
            "MatchState/AutonomousWinnerIsRed",
            MatchState.autonomousWinnerIsRed.isPresent()
                ? String.valueOf(MatchState.autonomousWinnerIsRed.get())
                : "No data");

    ShooterMath.setMorePow(m_wasAtTarget);
  }

  /** Returns whether the current state uses the m_intake. */
  @NotLogged
  public boolean currentStateUsesIntake() {
    return switch (m_currentState) {
      case ScoreWhileIntaking,
              ScoreWithIntakeUp,
              ScoreWithIntakeUpAuto,
              Intake,
              IntakeAuto,
              Override ->
          true;
      default -> false;
    };
  }

  /** Applies rumble feedback to the mechanism controller based on match state. */
  public void applyRumble() {
    if (m_restingVoltage == -1) {
      if (m_timer.hasElapsed(5) || DriverStation.isEnabled()) {
        m_restingVoltage = RobotController.getBatteryVoltage();
      } else {
        return;
      }
    }
    if (DriverStation.isTeleopEnabled() && m_restingVoltage != -1) {
      // Rumble battery sag to alert driver the level of "cooked" the battery is. Rumble is 0 when
      // battery is at resting voltage, and 1 when battery is at brownout voltage.
      m_driver.setRumble(
          RumbleType.kBothRumble,
          1.0
              - ((RobotController.getBatteryVoltage() - RobotController.getBrownoutVoltage())
                  / (m_restingVoltage - RobotController.getBrownoutVoltage())));
    } else {
      m_driver.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  /**
   * Handles the transitions from wanted states to current states
   *
   * @return the new current state
   */
  @NotLogged
  private CurrentStates handleStateTransitions() {
    return switch (m_wantedState) {
      case Default -> CurrentStates.Idle;
      case Intake -> CurrentStates.Intake;
      case Shoot ->
          passingRobotState()
              ? switch (m_intakeAddableState) {
                case IntakeUp -> CurrentStates.PassWithIntakeUp;
                case Intaking -> CurrentStates.PassWhileIntaking;
              }
              : switch (m_intakeAddableState) {
                case IntakeUp -> CurrentStates.ScoreWithIntakeUp;
                case Intaking -> CurrentStates.ScoreWhileIntaking;
              };
      case DefaultAuto -> CurrentStates.IdleAuto;
      case IntakeAuto -> CurrentStates.IntakeAuto;
      case ScoreAuto ->
          switch (m_intakeAddableState) {
            case IntakeUp -> CurrentStates.ScoreWithIntakeUpAuto;
            case Intaking -> CurrentStates.ScoreWhileIntakingAuto;
          };
      case Override -> CurrentStates.Override;
    };
  }

  /** Applies the current states to the subsystems */
  private void applyStates() {
    switch (m_currentState) {
      case Idle:
        idle();
        break;
      case ScoreWithIntakeUp:
        scoreWithIntakeUp();
        break;
      case PassWithIntakeUp:
        passWithIntakeUp();
        break;
      case Intake:
        intake();
        break;
      case ScoreWhileIntaking:
        scoreWhileIntaking();
        break;
      case PassWhileIntaking:
        passWhileIntaking();
        break;
      case IdleAuto:
        idleAuto();
        break;
      case ScoreWithIntakeUpAuto:
        scoreWithIntakeUpAuto();
        break;
      case IntakeAuto:
        intakeAuto();
        break;
      case ScoreWhileIntakingAuto:
        scoreWhileIntakingAuto();
        break;
      case Override:
        override();
        break;
    }
  }

  @NotLogged
  private boolean passingRobotState() {
    return ((!Alliance.redAlliance
            && m_drivetrain.getPose().getX() >= FieldConstants.kFarBumpDistanceFromDS.in(Meters))
        || (Alliance.redAlliance
            && m_drivetrain.getPose().getX()
                <= FieldConstants.kFieldLength
                    .minus(FieldConstants.kFarBumpDistanceFromDS)
                    .in(Meters)));
  }

  private void idle() {
    m_drivetrain.setState(DriveStates.DriverControlled);
    m_shooter.setState(passingRobotState() ? ShooterStates.IdlePass : ShooterStates.IdleScore);
    m_intake.setState(
        m_intakeAddableState == IntakeAddableStates.Intaking ? IntakeStates.Down : IntakeStates.Up);
    m_feeder.setState(FeederStates.Idle);
    m_indexer.setState(IndexerStates.Idle);

    m_driveAtTargetDebouncer.calculate(false);
    m_wasAtTarget = m_readyToShootDebouncer.calculate(false);
  }

  private void scoreWithIntakeUp() {
    m_drivetrain.setRotationTarget(getRotationForScore());
    m_drivetrain.setState(DriveStates.RotationLock);

    if (!m_wasAtTarget) {
      m_wasAtTarget = m_readyToShootDebouncer.calculate(readyToShoot());
    }

    m_shooter.setState(ShooterStates.Score);
    m_intake.setState(IntakeStates.UpAndIntake);
    m_feeder.setState(m_wasAtTarget ? FeederStates.Run : FeederStates.Idle);
    m_indexer.setState(m_wasAtTarget ? IndexerStates.Run : IndexerStates.Idle);
  }

  private void passWithIntakeUp() {
    m_drivetrain.setRotationTarget(getRotationForPass());
    m_drivetrain.setState(DriveStates.RotationLock);

    if (!m_wasAtTarget) {
      m_wasAtTarget = m_readyToShootDebouncer.calculate(readyToShoot());
    }

    m_shooter.setState(ShooterStates.Pass);
    m_intake.setState(IntakeStates.UpAndIntake);
    m_feeder.setState(m_wasAtTarget ? FeederStates.Run : FeederStates.Idle);
    m_indexer.setState(m_wasAtTarget ? IndexerStates.Run : IndexerStates.Idle);
  }

  private void intake() {
    m_drivetrain.setState(DriveStates.DriverControlled);
    m_shooter.setState(passingRobotState() ? ShooterStates.IdlePass : ShooterStates.IdleScore);
    m_intake.setState(IntakeStates.Intake);
    m_feeder.setState(FeederStates.Idle);
    m_indexer.setState(IndexerStates.Idle);

    m_driveAtTargetDebouncer.calculate(false);
    m_wasAtTarget = m_readyToShootDebouncer.calculate(false);
  }

  private void scoreWhileIntaking() {
    m_drivetrain.setRotationTarget(getRotationForScore());
    m_drivetrain.setState(DriveStates.RotationLock);

    if (!m_wasAtTarget) {
      m_wasAtTarget = m_readyToShootDebouncer.calculate(readyToShoot());
    }

    m_shooter.setState(ShooterStates.Score);
    m_intake.setState(IntakeStates.Intake);
    m_feeder.setState(m_wasAtTarget ? FeederStates.Run : FeederStates.Idle);
    m_indexer.setState(m_wasAtTarget ? IndexerStates.Run : IndexerStates.Idle);
  }

  private void passWhileIntaking() {
    m_drivetrain.setRotationTarget(getRotationForPass());
    m_drivetrain.setState(DriveStates.RotationLock);

    if (!m_wasAtTarget) {
      m_wasAtTarget = m_readyToShootDebouncer.calculate(readyToShoot());
    }

    m_shooter.setState(ShooterStates.Pass);
    m_intake.setState(IntakeStates.Intake);
    m_feeder.setState(m_wasAtTarget ? FeederStates.Run : FeederStates.Idle);
    m_indexer.setState(m_wasAtTarget ? IndexerStates.Run : IndexerStates.Idle);
  }

  private void idleAuto() {
    m_shooter.setState(ShooterStates.AutoPreset);
    m_feeder.setState(FeederStates.Idle);
    m_indexer.setState(IndexerStates.Idle);

    m_driveAtTargetDebouncer.calculate(false);
    m_wasAtTarget = m_readyToShootDebouncer.calculate(false);
  }

  private void scoreWhileIntakingAuto() {
    m_shooter.setState(ShooterStates.Score);
    m_intake.setState(IntakeStates.Intake);
    m_feeder.setState(readyToShoot() ? FeederStates.Run : FeederStates.Idle);
    m_indexer.setState(
        m_readyToShootDebouncer.calculate(readyToShoot()) ? IndexerStates.Run : IndexerStates.Idle);
  }

  private void scoreWithIntakeUpAuto() {
    m_shooter.setState(ShooterStates.Score);
    m_intake.setState(IntakeStates.UpAndIntake);
    m_feeder.setState(readyToShoot() ? FeederStates.Run : FeederStates.Idle);
    m_indexer.setState(
        m_readyToShootDebouncer.calculate(readyToShoot()) ? IndexerStates.Run : IndexerStates.Idle);
  }

  private void intakeAuto() {
    m_shooter.setState(ShooterStates.AutoPreset);
    m_intake.setState(IntakeStates.Intake);
    m_feeder.setState(FeederStates.Idle);
    m_indexer.setState(IndexerStates.Idle);

    m_driveAtTargetDebouncer.calculate(false);
    m_wasAtTarget = m_readyToShootDebouncer.calculate(false);
  }

  private void override() {
    m_intake.setState(
        m_intakeAddableState == IntakeAddableStates.IntakeUp
            ? IntakeStates.UpAndIntake
            : IntakeStates.Intake);
    m_feeder.setState(FeederStates.Idle);
    m_indexer.setState(IndexerStates.Run);

    m_driveAtTargetDebouncer.calculate(false);
    m_wasAtTarget = m_readyToShootDebouncer.calculate(false);
  }

  @Logged(importance = Importance.CRITICAL)
  public boolean drivetrainAtTarget() {
    return m_driveAtTargetDebouncer.calculate(
        (Alliance.redAlliance
                ? 180.0
                    - Math.abs(
                        m_drivetrain
                            .getRotation()
                            .minus(m_drivetrain.getRotationTarget())
                            .getDegrees())
                : Math.abs(
                    m_drivetrain
                        .getRotation()
                        .minus(m_drivetrain.getRotationTarget())
                        .getDegrees()))
            <= 10);
  }

  @NotLogged
  private boolean shooterAtTarget() {
    return m_shooter.isHoodAtTargetAngle();
  }

  @Logged(importance = Importance.CRITICAL)
  public boolean readyToShoot() {
    return drivetrainAtTarget() && shooterAtTarget();
  }

  @NotLogged
  public Rotation2d getRotationForScore() {
    return ShooterMath.currentSolution.robotHeading();
  }

  @NotLogged
  public Rotation2d getRotationForPass() {
    return ShooterMath.currentPassingSolution.robotHeading();
  }

  @NotLogged
  public boolean isStateTryingToShoot() {
    return switch (m_currentState) {
      case PassWhileIntaking,
              PassWithIntakeUp,
              ScoreWhileIntaking,
              ScoreWhileIntakingAuto,
              ScoreWithIntakeUp,
              ScoreWithIntakeUpAuto ->
          true;
      default -> false;
    };
  }

  /** The wanted states of superstructure */
  public enum WantedStates {
    Default(),
    Intake(),
    Shoot(),

    DefaultAuto(),
    IntakeAuto(),
    ScoreAuto(),

    Override()
  }

  /** The current states of superstructure */
  public enum CurrentStates {
    Idle(),
    Intake(),
    ScoreWhileIntaking(),
    ScoreWithIntakeUp(),
    PassWhileIntaking(),
    PassWithIntakeUp(),

    IdleAuto(),
    IntakeAuto(),
    ScoreWhileIntakingAuto(),
    ScoreWithIntakeUpAuto(),

    Override()
  }

  /** The addable states of intake */
  public enum IntakeAddableStates {
    Intaking(),
    IntakeUp()
  }

  /** The addable states of intake */
  public enum ShooterSubStates {
    Score(),
    Pass()
  }

  /**
   * @param state the wanted state to set
   */
  public void setWantedState(WantedStates state) {
    m_wantedState = state;
  }

  /**
   * @param state the addable state to set
   */
  public void setIntakeAddableState(IntakeAddableStates state) {
    m_intakeAddableState = state;
  }

  /**
   * @param state the wanted state to set
   * @return a command that sets the wanted state
   */
  @NotLogged
  public Command setWantedStateCommand(WantedStates state) {
    return Commands.runOnce(() -> setWantedState(state)).ignoringDisable(true);
  }

  /**
   * @param state the addable state to set
   * @return a command that sets the addable state
   */
  @NotLogged
  public Command setIntakeAddableStateCommand(IntakeAddableStates state) {
    return Commands.runOnce(() -> setIntakeAddableState(state)).ignoringDisable(true);
  }

  /** Returns the current state of the superstructure. */
  @NotLogged
  public CurrentStates getCurrentState() {
    return m_currentState;
  }
}
