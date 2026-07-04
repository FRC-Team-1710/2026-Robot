package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.MatchState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.Feeder.FEEDER_STATE;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.SHOOTER_STATE;
import frc.robot.utils.MathUtils;
import frc.robot.utils.shooterMath.ShooterMath4;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  private CommandXboxController m_driver;
  private CommandXboxController m_mech;
  private CommandSwerveDrivetrain m_drivetrain;
  private Intake m_intake;
  private Shooter m_shooter;
  private Indexer m_indexer;
  private Feeder m_feeder;

  private WantedStates m_wantedState = WantedStates.Default;

  private CurrentStates m_currentState = CurrentStates.Idle;

  private boolean m_didIntake = false;

  private IntakeAddableStates m_intakeAddableState = IntakeAddableStates.Intaking;

  private ShooterAddableStates m_shooterAddableState = ShooterAddableStates.Idle;

  private final Debouncer m_debouncer = new Debouncer(0.01);

  private final Debouncer m_debouncerDrive = new Debouncer(0.5);

  private boolean m_wasAtTarget = false;

  /**
   * Constructs the superstructure with all subsystem references.
   *
   * @param driver the driver controller
   * @param mech the mechanism controller
   * @param m_drivetrain the swerve m_drivetrain
   * @param m_intake the m_intake subsystem
   * @param m_shooter the m_shooter subsystem
   * @param m_indexer the m_indexer subsystem
   * @param m_feeder the m_feeder subsystem
   */
  public Superstructure(
      CommandXboxController driver,
      CommandXboxController mech,
      CommandSwerveDrivetrain m_drivetrain,
      Intake m_intake,
      Shooter m_shooter,
      Indexer m_indexer,
      Feeder m_feeder) {
    this.m_driver = driver;
    this.m_mech = mech;
    this.m_drivetrain = m_drivetrain;
    this.m_intake = m_intake;
    this.m_shooter = m_shooter;
    this.m_indexer = m_indexer;
    this.m_feeder = m_feeder;
  }

  /** Runs periodic logic for state transitions and subsystem coordination. */
  public void periodic() {
    m_currentState = handleStateTransitions();
    applyStates();

    applyRumble();

    Logger.recordOutput("Superstructure/AllianceRed", Alliance.redAlliance);
    Logger.recordOutput(
        "MatchState/TimeTillActive",
        Math.round(MatchState.timeTillActive().in(Seconds) * 10.0) / 10.0);
    Logger.recordOutput(
        "MatchState/TimeTillInactive",
        Math.round(MatchState.timeTillInactive().in(Seconds) * 10.0) / 10.0);
    Logger.recordOutput(
        "MatchState/AutonomousWinnerIsRed",
        MatchState.autonomousWinnerIsRed.isPresent()
            ? String.valueOf(MatchState.autonomousWinnerIsRed.get())
            : "No data");
    Logger.recordOutput("MatchState/IsActive", MatchState.isActive());
    Logger.recordOutput("Superstructure/WantedState", m_wantedState);
    Logger.recordOutput("Superstructure/CurrentState", m_currentState);
    Logger.recordOutput("Superstructure/DidIntake", m_didIntake);
    Logger.recordOutput("Superstructure/IntakeAddableState", m_intakeAddableState);
    Logger.recordOutput("Superstructure/ShooterAddableState", m_shooterAddableState);
    Logger.recordOutput("Superstructure/WasAtTarget", m_wasAtTarget);
  }

  /** Returns whether the current state uses the m_intake. */
  public boolean currentStateUsesIntake() {
    return switch (m_currentState) {
      case Score -> true;
      case ScoreWhileIntaking -> true;
      case ScoreWithIntakeUp -> true;
      case ScoreWhileIntakingAuto -> true;
      case ScoreWithIntakeUpAuto -> true;
      case Intake -> true;
      case IntakeAuto -> true;
      case Override -> true;
      default -> false;
    };
  }

  /** Returns whether the current state does not use the m_intake. */
  public boolean currentStateDoesntUseIntake() {
    return !currentStateUsesIntake();
  }

  /** Applies rumble feedback to the mechanism controller based on match state. */
  public void applyRumble() {
    var timeUntilRumble = MatchState.timeTillActive().plus(MatchState.timeTillInactive());
    if (!DriverStation.isAutonomous() && MatchState.autonomousWinnerIsRed.isPresent()) {
      if (!timeUntilRumble.isEquivalent(Seconds.of(0))) {
        m_mech.setRumble(RumbleType.kBothRumble, timeUntilRumble.in(Seconds) < 2.5 ? 1 : 0);
      } else {
        m_mech.setRumble(RumbleType.kBothRumble, 0);
      }
    } else if (!MatchState.autonomousWinnerIsRed.isPresent()) {
      m_mech.setRumble(RumbleType.kBothRumble, 1);
    }
  }

  /**
   * Handles the transitions from wanted states to current states
   *
   * @return the new current state
   */
  private CurrentStates handleStateTransitions() {
    return switch (m_wantedState) {
      case Default -> CurrentStates.Idle;
      case Shoot ->
          ((!Alliance.redAlliance
                      && m_drivetrain.getPose().getX()
                          >= FieldConstants.kBumpDistanceFromDS.in(Meters))
                  || (Alliance.redAlliance
                      && m_drivetrain.getPose().getX()
                          <= FieldConstants.kFieldLength
                              .minus(FieldConstants.kBumpDistanceFromDS)
                              .in(Meters)))
              ? switch (m_intakeAddableState) {
                case Jostle -> CurrentStates.Shoot;
                case IntakeUp -> CurrentStates.ShootWithIntakeUp;
                case Intaking -> CurrentStates.ShootWhileIntaking;
              }
              : switch (m_intakeAddableState) {
                case Jostle -> CurrentStates.Score;
                case IntakeUp -> CurrentStates.ScoreWithIntakeUp;
                case Intaking -> CurrentStates.ScoreWhileIntaking;
              };
      case Intake -> CurrentStates.Intake;
      case IntakeAndShoot ->
          ((!Alliance.redAlliance
                      && m_drivetrain.getPose().getX()
                          >= FieldConstants.kBumpDistanceFromDS.in(Meters))
                  || (Alliance.redAlliance
                      && m_drivetrain.getPose().getX()
                          <= FieldConstants.kFieldLength
                              .minus(FieldConstants.kBumpDistanceFromDS)
                              .in(Meters)))
              ? CurrentStates.ShootWhileIntaking
              : CurrentStates.ScoreWhileIntaking;
      case Climb -> CurrentStates.Climb;
      case DefaultAuto -> CurrentStates.IdleAuto;
      case ShootAuto ->
          switch (m_intakeAddableState) {
            case Jostle -> CurrentStates.ScoreAuto;
            case IntakeUp -> CurrentStates.ScoreWithIntakeUpAuto;
            case Intaking -> CurrentStates.ScoreWhileIntakingAuto;
          };
      case IntakeAuto -> CurrentStates.IntakeAuto;
      case IntakeAndShootAuto -> CurrentStates.ScoreWhileIntakingAuto;
      case ClimbAuto -> CurrentStates.ClimbAuto;
      case Override -> CurrentStates.Override;
    };
  }

  /** Applies the current states to the subsystems */
  private void applyStates() {
    switch (m_currentState) {
      case Idle:
        idle();
        break;
      case Score:
        score();
        break;
      case ScoreWithIntakeUp:
        scoreWithIntakeUp();
        break;
      case Shoot:
        shoot();
        break;
      case ShootWithIntakeUp:
        shootWithIntakeUp();
        break;
      case Intake:
        intake();
        break;
      case ScoreWhileIntaking:
        scoreWhileIntaking();
        break;
      case ShootWhileIntaking:
        shootWhileIntaking();
        break;
      case Climb:
        climb();
        break;
      case IdleAuto:
        idleAuto();
        break;
      case ScoreAuto:
        scoreAuto();
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
      case ClimbAuto:
        climbAuto();
        break;
      case Override:
        override();
        break;
    }
  }

  private void idle() {
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.DRIVER_CONTROLLED);
    // if (m_drivetrain.fieldCentric.currentDriveState == RequestStates.BUMP_ASSIST
    //     && m_drivetrain.fieldCentric.isGoingToAllianceZone()) {
    //   m_shooter.setState(SHOOTER_STATE.SHOOT); // Get ready before getting there
    // } else {
    m_shooter.setState(
        m_shooterAddableState == ShooterAddableStates.Idle
            ? SHOOTER_STATE.IDLE
            : SHOOTER_STATE.SHOOT);
    // }
    m_indexer.setState(IndexStates.Idle);
    m_feeder.setState(FEEDER_STATE.STOP);
    // if (m_drivetrain.fieldCentric.shouldRaiseIntake()) {
    //   m_intake.setState(IntakeStates.Half);
    // }
    m_wasAtTarget = m_debouncer.calculate(false);
  }

  private void score() {
    // if (!driveAtTarget() || !DrivetrainAutomationConstants.BumpDetection.shouldAlignBump()) {
    m_drivetrain.setRotationTarget(getRotationForScore());
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);
    // } else {
    //   m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.X_LOCK);
    // }

    if (!m_wasAtTarget) {
      m_wasAtTarget = m_debouncer.calculate(flywheelAtTargetWithWait());
    }

    m_intake.setState(IntakeStates.Jostle);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_feeder.setState(m_wasAtTarget ? FEEDER_STATE.FEEDING : FEEDER_STATE.STOP);
    m_indexer.setState(m_wasAtTarget ? IndexStates.Indexing : IndexStates.Idle);

    m_didIntake = false;
  }

  private void scoreWithIntakeUp() {
    // if (!driveAtTarget() || !DrivetrainAutomationConstants.BumpDetection.shouldAlignBump()) {
    m_drivetrain.setRotationTarget(getRotationForScore());
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);
    // } else {
    //   m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.X_LOCK);
    // }

    if (!m_wasAtTarget) {
      m_wasAtTarget = m_debouncer.calculate(flywheelAtTargetWithWait());
    }

    m_intake.setState(IntakeStates.UpAndIntake);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_feeder.setState(m_wasAtTarget ? FEEDER_STATE.FEEDING : FEEDER_STATE.STOP);
    m_indexer.setState(m_wasAtTarget ? IndexStates.Indexing : IndexStates.Idle);

    m_didIntake = false;
  }

  private void shoot() {
    // if (!driveAtTarget() || !DrivetrainAutomationConstants.BumpDetection.shouldAlignBump()) {
    m_drivetrain.setRotationTarget(getRotationForShoot());
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);
    // } else {
    //   m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.X_LOCK);
    // }

    if (!m_wasAtTarget) {
      m_wasAtTarget = m_debouncer.calculate(flywheelAtTargetWithWait());
    }

    m_intake.setState(IntakeStates.Jostle);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_feeder.setState(m_wasAtTarget ? FEEDER_STATE.FEEDING : FEEDER_STATE.STOP);
    m_indexer.setState(m_wasAtTarget ? IndexStates.Indexing : IndexStates.Idle);

    m_didIntake = false;
  }

  private void shootWithIntakeUp() {
    // if (!driveAtTarget() || !DrivetrainAutomationConstants.BumpDetection.shouldAlignBump()) {
    m_drivetrain.setRotationTarget(getRotationForShoot());
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);
    // } else {
    //   m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.X_LOCK);
    // }

    if (!m_wasAtTarget) {
      m_wasAtTarget = m_debouncer.calculate(flywheelAtTargetWithWait());
    }

    m_intake.setState(IntakeStates.UpAndIntake);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_feeder.setState(m_wasAtTarget ? FEEDER_STATE.FEEDING : FEEDER_STATE.STOP);
    m_indexer.setState(m_wasAtTarget ? IndexStates.Indexing : IndexStates.Idle);

    m_didIntake = false;
  }

  private void intake() {
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.DRIVER_CONTROLLED);
    m_intake.setState(IntakeStates.Intaking);
    m_shooter.setState(
        m_shooterAddableState == ShooterAddableStates.Idle
            ? SHOOTER_STATE.IDLE
            : SHOOTER_STATE.SHOOT);
    m_indexer.setState(IndexStates.Idle);
    m_feeder.setState(FEEDER_STATE.STOP);

    m_didIntake = true;
    m_wasAtTarget = m_debouncer.calculate(false);
  }

  private void scoreWhileIntaking() {
    // if (!driveAtTarget() || !DrivetrainAutomationConstants.BumpDetection.shouldAlignBump()) {
    m_drivetrain.setRotationTarget(getRotationForScore());
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);
    // } else {
    //   m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.X_LOCK);
    // }

    if (!m_wasAtTarget) {
      m_wasAtTarget = m_debouncer.calculate(flywheelAtTargetWithWait());
    }

    m_intake.setState(IntakeStates.Intaking);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_feeder.setState(m_wasAtTarget ? FEEDER_STATE.FEEDING : FEEDER_STATE.STOP);
    m_indexer.setState(m_wasAtTarget ? IndexStates.Indexing : IndexStates.Idle);

    m_didIntake = false;
  }

  private void shootWhileIntaking() {
    // if (!driveAtTarget() || !DrivetrainAutomationConstants.BumpDetection.shouldAlignBump()) {
    m_drivetrain.setRotationTarget(getRotationForShoot());
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);
    // } else {
    //   m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.X_LOCK);
    // }

    if (!m_wasAtTarget) {
      m_wasAtTarget = m_debouncer.calculate(flywheelAtTargetWithWait());
    }

    m_intake.setState(IntakeStates.Intaking);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_feeder.setState(m_wasAtTarget ? FEEDER_STATE.FEEDING : FEEDER_STATE.STOP);
    m_indexer.setState(m_wasAtTarget ? IndexStates.Indexing : IndexStates.Idle);

    m_didIntake = false;
  }

  private void climb() {
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.DRIVER_CONTROLLED);
    m_shooter.setState(
        m_shooterAddableState == ShooterAddableStates.Idle
            ? SHOOTER_STATE.IDLE
            : SHOOTER_STATE.SHOOT);
    m_indexer.setState(IndexStates.Idle);
    m_feeder.setState(FEEDER_STATE.STOP);
    m_wasAtTarget = m_debouncer.calculate(false);
  }

  private void idleAuto() {
    m_feeder.setState(FEEDER_STATE.STOP);
    m_indexer.setState(IndexStates.Idle);
    m_shooter.setState(
        m_shooterAddableState == ShooterAddableStates.Idle
            ? SHOOTER_STATE.IDLE
            : SHOOTER_STATE.SHOOT);
  }

  private void scoreAuto() {
    m_intake.setState(IntakeStates.Jostle);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_feeder.setState(flywheelAtTarget() ? FEEDER_STATE.FEEDING : FEEDER_STATE.STOP);
    m_indexer.setState(
        m_debouncer.calculate(flywheelAtTargetWithWait())
            ? IndexStates.Indexing
            : IndexStates.Idle);

    m_didIntake = false;
  }

  private void scoreWithIntakeUpAuto() {
    m_intake.setState(IntakeStates.UpAndIntake);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_feeder.setState(flywheelAtTarget() ? FEEDER_STATE.FEEDING : FEEDER_STATE.STOP);
    m_indexer.setState(
        m_debouncer.calculate(flywheelAtTargetWithWait())
            ? IndexStates.Indexing
            : IndexStates.Idle);

    m_didIntake = false;
  }

  private void intakeAuto() {
    m_intake.setState(IntakeStates.IntakingAuto);
    m_shooter.setState(SHOOTER_STATE.IDLE);
    m_indexer.setState(IndexStates.Idle);
    m_feeder.setState(FEEDER_STATE.STOP);

    m_didIntake = true;
  }

  private void scoreWhileIntakingAuto() {
    m_intake.setState(IntakeStates.Intaking);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_feeder.setState(flywheelAtTarget() ? FEEDER_STATE.FEEDING : FEEDER_STATE.STOP);
    m_indexer.setState(
        m_debouncer.calculate(flywheelAtTargetWithWait())
            ? IndexStates.Indexing
            : IndexStates.Idle);

    m_didIntake = false;
  }

  private void climbAuto() {
    m_shooter.setState(SHOOTER_STATE.IDLE);
    m_indexer.setState(IndexStates.Idle);
    m_feeder.setState(FEEDER_STATE.STOP);
  }

  private void override() {
    m_intake.setState(
        m_intakeAddableState == IntakeAddableStates.IntakeUp
            ? IntakeStates.UpAndIntake
            : IntakeStates.Jostle);
    m_indexer.setState(IndexStates.Indexing);
    m_feeder.setState(FEEDER_STATE.FEEDING);
  }

  /**
   * @return whether the superstructure is currently in a shooting (not scoring) state
   */
  public boolean shooting() {
    return m_currentState == CurrentStates.Shoot
        || m_currentState == CurrentStates.ShootWhileIntaking
        || m_currentState == CurrentStates.ShootWithIntakeUp;
  }

  public boolean driveAtTarget() {
    return m_debouncerDrive.calculate(
        Math.abs(
                m_drivetrain
                    .getRotation()
                    .minus(m_drivetrain.fieldCentric.rotationTarget)
                    .getDegrees())
            <= 10);
  }

  public boolean flywheelAtTarget() {
    return m_shooter.isAtTargetVelocity() && m_shooter.isHoodAtTargetAngle() && driveAtTarget();
  }

  /** Returns whether the m_shooter is at its target with wait. */
  public boolean flywheelAtTargetWithWait() {
    return flywheelAtTarget();
    // && MatchState.canShoot(ShooterMath4.currentSolution.tof().in(Seconds));
  }

  public Rotation2d getRotationForScore() {
    return ShooterMath4.currentSolution.robotHeading().plus(Rotation2d.k180deg);
  }

  public Rotation2d getRotationForShoot() {
    return !Alliance.redAlliance
        ? (m_drivetrain.getPose().getY() <= FieldConstants.kHubCornerNeutralZone1.getY()
                && m_drivetrain.getPose().getY() >= FieldConstants.kHubCornerNeutralZone2.getY()
            ? m_drivetrain
                .getPose()
                .getTranslation()
                .minus(
                    MathUtils.getClosest(
                        m_drivetrain.getPose().getTranslation(),
                        FieldConstants.kHubCornerNeutralZone1,
                        FieldConstants.kHubCornerNeutralZone2))
                .getAngle()
            : Rotation2d.kZero)
        : (m_drivetrain.getPose().getY() <= FieldConstants.kHubCornerNeutralZone1.getY()
                && m_drivetrain.getPose().getY() >= FieldConstants.kHubCornerNeutralZone2.getY()
            ? m_drivetrain
                .getPose()
                .getTranslation()
                .minus(
                    MathUtils.getClosest(
                        m_drivetrain.getPose().getTranslation(),
                        MathUtils.opposite(FieldConstants.kHubCornerNeutralZone1),
                        MathUtils.opposite(FieldConstants.kHubCornerNeutralZone2)))
                .getAngle()
            : Rotation2d.k180deg);
  }

  public boolean isStateTryingToShoot() {
    return switch (m_currentState) {
      case Shoot,
              ShootWithIntakeUp,
              ShootWhileIntaking,
              Score,
              ScoreWithIntakeUp,
              ScoreWhileIntaking,
              ScoreAuto,
              ScoreWithIntakeUpAuto,
              ScoreWhileIntakingAuto ->
          true;
      default -> false;
    };
  }

  /** The wanted states of superstructure */
  public enum WantedStates {
    // DO NOT RENAME (unless AutosChooser is updated as well)
    Default(),
    Shoot(),
    Intake(),
    IntakeAndShoot(),
    Climb(),
    DefaultAuto(),
    ShootAuto(),
    IntakeAuto(),
    IntakeAndShootAuto(),
    ClimbAuto(),
    Override(),
  }

  /** The current states of superstructure */
  public enum CurrentStates {
    Idle(),
    Score(),
    ScoreWithIntakeUp(),
    Shoot(),
    ShootWithIntakeUp(),
    Intake(),
    ScoreWhileIntaking(),
    ShootWhileIntaking(),
    Climb(),
    IdleAuto(),
    ScoreAuto(),
    ScoreWithIntakeUpAuto(),
    IntakeAuto(),
    ScoreWhileIntakingAuto(),
    ClimbAuto(),
    Override()
  }

  /** The addable states of intake */
  public enum IntakeAddableStates {
    Intaking(),
    Jostle(),
    IntakeUp()
  }

  /** The addable states of shooter */
  public enum ShooterAddableStates {
    SpinUp(),
    Idle()
  }

  /**
   * @param state the wanted state to set
   */
  public void setWantedState(WantedStates state) {
    m_wantedState = state;
    Logger.recordOutput("Superstructure/WantedState", state);
  }

  /**
   * @param state the addable state to set
   */
  public void setIntakeAddableState(IntakeAddableStates state) {
    m_intakeAddableState = state;
    Logger.recordOutput("Superstructure/IntakeAddableState", state);
  }

  /**
   * @param state the addable state to set
   */
  public void setShooterAddableState(ShooterAddableStates state) {
    m_shooterAddableState = state;
    Logger.recordOutput("Superstructure/ShooterAddableState", state);
  }

  /**
   * @param state the wanted state to set
   * @return a command that sets the wanted state
   */
  public Command setWantedStateCommand(WantedStates state) {
    return Commands.runOnce(() -> setWantedState(state)).ignoringDisable(true);
  }

  /**
   * @param state the addable state to set
   * @return a command that sets the addable state
   */
  public Command setIntakeAddableStateCommand(IntakeAddableStates state) {
    return Commands.runOnce(() -> setIntakeAddableState(state)).ignoringDisable(true);
  }

  /**
   * @param state the addable state to set
   * @return a command that sets the addable state
   */
  public Command setShooterAddableStateCommand(ShooterAddableStates state) {
    return Commands.runOnce(() -> setShooterAddableState(state)).ignoringDisable(true);
  }

  /** Returns the current state of the superstructure. */
  public CurrentStates getCurrentState() {
    return m_currentState;
  }
}
