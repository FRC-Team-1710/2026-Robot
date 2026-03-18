package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.constants.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.MatchState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.Feeder.FEEDER_STATE;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.SHOOTER_STATE;
import frc.robot.utils.MathUtils;
import frc.robot.utils.shooterMath.ShooterMath2;

@Logged
public class Superstructure {
  @NotLogged private CommandXboxController m_driver;
  @NotLogged private CommandXboxController m_mech;
  @NotLogged private CommandSwerveDrivetrain m_drivetrain;
  @NotLogged private Intake m_intake;
  @NotLogged private Shooter m_shooter;
  @NotLogged private Indexer m_indexer;
  @NotLogged private Feeder m_feeder;
  @NotLogged private Leds m_leds;

  @Logged(importance = Importance.CRITICAL)
  private WantedStates m_wantedState = WantedStates.Default;

  @Logged(importance = Importance.CRITICAL)
  private CurrentStates m_currentState = CurrentStates.Idle;

  @Logged(importance = Importance.INFO)
  private boolean m_didIntake = false;

  @Logged(importance = Importance.CRITICAL)
  private AddableStates m_addableState = AddableStates.Jostle;

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
      Feeder m_feeder,
      Leds leds) {
    this.m_driver = driver;
    this.m_mech = mech;
    this.m_drivetrain = m_drivetrain;
    this.m_intake = m_intake;
    this.m_shooter = m_shooter;
    this.m_indexer = m_indexer;
    this.m_feeder = m_feeder;
    this.m_leds = leds;
  }

  /** Runs periodic logic for state transitions and subsystem coordination. */
  public void periodic() {
    m_shooter.setGoingTowardsAllianceZone(m_drivetrain.isGoingTowardsAllianceZone());
    m_shooter.setDidIntake(m_didIntake);

    m_currentState = handleStateTransitions();
    applyStates();

    applyRumble();

    Robot.telemetry().log("redAlliance", Alliance.redAlliance);

    Robot.telemetry().log("MatchState/TimeTillActive", MatchState.timeTillActive());
    Robot.telemetry().log("MatchState/TimeTillInactive", MatchState.timeTillInactive());
    Robot.telemetry()
        .log(
            "MatchState/AutonomousWinnerIsRed",
            MatchState.autonomousWinnerIsRed.isPresent()
                ? String.valueOf(MatchState.autonomousWinnerIsRed.get())
                : "No data");
    Robot.telemetry().log("MatchState/IsActive", MatchState.isActive());
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
      case IntakeWithVision -> true;
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
  @NotLogged
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
              ? switch (m_addableState) {
                case Jostle -> CurrentStates.Shoot;
                case IntakeUp -> CurrentStates.ShootWithIntakeUp;
                case Intaking -> CurrentStates.ShootWhileIntaking;
              }
              : switch (m_addableState) {
                case Jostle -> CurrentStates.Score;
                case IntakeUp -> CurrentStates.ScoreWithIntakeUp;
                case Intaking -> CurrentStates.ScoreWhileIntaking;
              };
      case Intake -> CurrentStates.Intake;
      case IntakeWithVision -> CurrentStates.IntakeWithVision;
      case IntakeAndShoot -> CurrentStates.ScoreWhileIntaking;
      case Climb -> CurrentStates.Climb;
      case DefaultAuto -> CurrentStates.IdleAuto;
      case ShootAuto ->
          switch (m_addableState) {
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
      case IntakeWithVision:
        intakeWithVision();
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
    m_shooter.setState(SHOOTER_STATE.IDLE);
    m_indexer.setState(IndexStates.Idle);
    m_feeder.setState(FEEDER_STATE.STOP);
    if (m_drivetrain.fieldCentric.shouldRaiseIntake()) {
      m_intake.setState(IntakeStates.Half);
    }
  }

  private void score() {
    m_drivetrain.setRotationTarget(getRotationForScore());
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);
    m_intake.setState(IntakeStates.Jostle);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_indexer.setState(anyAtTargetWithWait() ? IndexStates.Indexing : IndexStates.Idle);
    m_feeder.setState(
        allAtTargetWithWait()
            ? FEEDER_STATE.FEEDING
            : leftAtTargetWithWait()
                ? FEEDER_STATE.FEEDING_LEFT
                : rightAtTargetWithWait() ? FEEDER_STATE.FEEDING_RIGHT : FEEDER_STATE.STOP);

    m_didIntake = false;
  }

  private void scoreWithIntakeUp() {
    m_drivetrain.setRotationTarget(getRotationForScore());
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);
    m_intake.setState(IntakeStates.Up);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_indexer.setState(anyAtTargetWithWait() ? IndexStates.Indexing : IndexStates.Idle);
    m_feeder.setState(
        allAtTargetWithWait()
            ? FEEDER_STATE.FEEDING
            : leftAtTargetWithWait()
                ? FEEDER_STATE.FEEDING_LEFT
                : rightAtTargetWithWait() ? FEEDER_STATE.FEEDING_RIGHT : FEEDER_STATE.STOP);

    m_didIntake = false;
  }

  private void shoot() {
    m_drivetrain.setRotationTarget(getRotationForShoot());
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);
    m_intake.setState(IntakeStates.Jostle);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_indexer.setState(anyAtTarget() ? IndexStates.Indexing : IndexStates.Idle);
    m_feeder.setState(
        allAtTarget()
            ? FEEDER_STATE.FEEDING
            : leftAtTarget()
                ? FEEDER_STATE.FEEDING_LEFT
                : rightAtTarget() ? FEEDER_STATE.FEEDING_RIGHT : FEEDER_STATE.STOP);

    m_didIntake = false;
  }

  private void shootWithIntakeUp() {
    m_drivetrain.setRotationTarget(getRotationForShoot());
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);
    m_intake.setState(IntakeStates.Up);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_indexer.setState(anyAtTarget() ? IndexStates.Indexing : IndexStates.Idle);
    m_feeder.setState(
        allAtTarget()
            ? FEEDER_STATE.FEEDING
            : leftAtTarget()
                ? FEEDER_STATE.FEEDING_LEFT
                : rightAtTarget() ? FEEDER_STATE.FEEDING_RIGHT : FEEDER_STATE.STOP);

    m_didIntake = false;
  }

  private void intake() {
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.DRIVER_CONTROLLED);
    m_intake.setState(IntakeStates.Intaking);
    m_shooter.setState(SHOOTER_STATE.IDLE);
    m_indexer.setState(IndexStates.Idle);
    m_feeder.setState(FEEDER_STATE.STOP);

    m_didIntake = true;
  }

  private void intakeWithVision() {
    m_drivetrain.setRotationTarget(
        m_drivetrain
            .getPose()
            .getRotation()
            .minus(new Rotation2d(m_drivetrain.getFuelTargetYaw())));
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);
    m_intake.setState(IntakeStates.Intaking);
    m_shooter.setState(SHOOTER_STATE.IDLE);
    m_indexer.setState(IndexStates.Idle);
    m_feeder.setState(FEEDER_STATE.STOP);

    m_didIntake = true;
  }

  private void scoreWhileIntaking() {
    m_drivetrain.setRotationTarget(getRotationForScore());
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);
    m_intake.setState(IntakeStates.Intaking);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_indexer.setState(anyAtTargetWithWait() ? IndexStates.Indexing : IndexStates.Idle);
    m_feeder.setState(
        allAtTargetWithWait()
            ? FEEDER_STATE.FEEDING
            : leftAtTargetWithWait()
                ? FEEDER_STATE.FEEDING_LEFT
                : rightAtTargetWithWait() ? FEEDER_STATE.FEEDING_RIGHT : FEEDER_STATE.STOP);

    m_didIntake = false;
  }

  private void shootWhileIntaking() {
    m_drivetrain.setRotationTarget(getRotationForShoot());
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);
    m_intake.setState(IntakeStates.Intaking);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_indexer.setState(anyAtTarget() ? IndexStates.Indexing : IndexStates.Idle);
    m_feeder.setState(
        allAtTarget()
            ? FEEDER_STATE.FEEDING
            : leftAtTarget()
                ? FEEDER_STATE.FEEDING_LEFT
                : rightAtTarget() ? FEEDER_STATE.FEEDING_RIGHT : FEEDER_STATE.STOP);

    m_didIntake = false;
  }

  private void climb() {
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.DRIVER_CONTROLLED);
    m_shooter.setState(SHOOTER_STATE.IDLE);
    m_indexer.setState(IndexStates.Idle);
    m_feeder.setState(FEEDER_STATE.STOP);
  }

  private void idleAuto() {
    m_feeder.setState(FEEDER_STATE.STOP);
    m_shooter.setState(SHOOTER_STATE.IDLE);
    m_indexer.setState(IndexStates.Idle);
  }

  private void scoreAuto() {
    m_intake.setState(IntakeStates.Jostle);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_indexer.setState(anyAtTarget() ? IndexStates.Indexing : IndexStates.Idle);
    m_feeder.setState(
        allAtTarget()
            ? FEEDER_STATE.FEEDING
            : leftAtTarget()
                ? FEEDER_STATE.FEEDING_LEFT
                : rightAtTarget() ? FEEDER_STATE.FEEDING_RIGHT : FEEDER_STATE.STOP);

    m_didIntake = false;
  }

  private void scoreWithIntakeUpAuto() {
    m_intake.setState(IntakeStates.Up);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_indexer.setState(anyAtTarget() ? IndexStates.Indexing : IndexStates.Idle);
    m_feeder.setState(
        allAtTarget()
            ? FEEDER_STATE.FEEDING
            : leftAtTarget()
                ? FEEDER_STATE.FEEDING_LEFT
                : rightAtTarget() ? FEEDER_STATE.FEEDING_RIGHT : FEEDER_STATE.STOP);

    m_didIntake = false;
  }

  private void intakeAuto() {
    m_intake.setState(IntakeStates.Intaking);
    m_shooter.setState(SHOOTER_STATE.IDLE);
    m_indexer.setState(IndexStates.Idle);
    m_feeder.setState(FEEDER_STATE.STOP);

    m_didIntake = true;
  }

  private void scoreWhileIntakingAuto() {
    m_intake.setState(IntakeStates.Intaking);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_indexer.setState(anyAtTarget() ? IndexStates.Indexing : IndexStates.Idle);
    m_feeder.setState(
        allAtTarget()
            ? FEEDER_STATE.FEEDING
            : leftAtTarget()
                ? FEEDER_STATE.FEEDING_LEFT
                : rightAtTarget() ? FEEDER_STATE.FEEDING_RIGHT : FEEDER_STATE.STOP);

    m_didIntake = false;
  }

  private void climbAuto() {
    m_shooter.setState(SHOOTER_STATE.IDLE);
    m_indexer.setState(IndexStates.Idle);
    m_feeder.setState(FEEDER_STATE.STOP);
  }

  private void override() {
    m_intake.setState(
        m_addableState == AddableStates.IntakeUp ? IntakeStates.Up : IntakeStates.Jostle);
    m_indexer.setState(IndexStates.Indexing);
    m_feeder.setState(FEEDER_STATE.FEEDING);
  }

  /** Returns whether all shooters are at their targets. */
  @Logged(importance = Importance.INFO)
  public boolean allAtTarget() {
    return leftAtTarget() && rightAtTarget();
  }

  /** Returns whether all shooters are at their targets with wait. */
  @Logged(importance = Importance.INFO)
  public boolean allAtTargetWithWait() {
    return leftAtTargetWithWait() && rightAtTargetWithWait();
  }

  /** Returns whether any m_shooter is at its target. */
  @NotLogged
  public boolean anyAtTarget() {
    return leftAtTarget() || rightAtTarget();
  }

  /** Returns whether any m_shooter is at its target with wait. */
  @NotLogged
  public boolean anyAtTargetWithWait() {
    return leftAtTargetWithWait() || rightAtTargetWithWait();
  }

  @Logged(importance = Importance.CRITICAL)
  public boolean driveAtTarget() {
    return Math.abs(
            m_drivetrain.getRotation().minus(m_drivetrain.fieldCentric.rotationTarget).getDegrees())
        <= 5;
  }

  @Logged(importance = Importance.CRITICAL)
  public boolean leftAtTarget() {
    return m_shooter.isAtLeftTargetVelocity()
        && m_shooter.isHoodAtLeftTargetAngle()
        && driveAtTarget();
  }

  /** Returns whether the right m_shooter is at its target. */
  @Logged(importance = Importance.CRITICAL)
  public boolean rightAtTarget() {
    return m_shooter.isAtRightTargetVelocity()
        && m_shooter.isHoodAtRightTargetAngle()
        && driveAtTarget();
  }

  /** Returns whether the left m_shooter is at its target with wait. */
  @Logged(importance = Importance.CRITICAL)
  public boolean leftAtTargetWithWait() {
    return leftAtTarget()
        && MatchState.canShoot(ShooterMath2.currentSolution.shooterLeft().tof().in(Seconds));
  }

  /** Returns whether the right m_shooter is at its target with wait. */
  @Logged(importance = Importance.CRITICAL)
  public boolean rightAtTargetWithWait() {
    return rightAtTarget()
        && MatchState.canShoot(ShooterMath2.currentSolution.shooterRight().tof().in(Seconds));
  }

  @NotLogged
  public Rotation2d getRotationForScore() {
    return ShooterMath2.currentSolution.robotHeading().plus(Rotation2d.k180deg);
  }

  @NotLogged
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

  /** The wanted states of superstructure */
  public enum WantedStates {
    // DO NOT RENAME (unless AutosChooser is updated as well)
    Default(),
    Shoot(),
    Intake(),
    IntakeWithVision(),
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
    IntakeWithVision(),
    ScoreWhileIntaking(),
    ShootWhileIntaking(),
    Climb(),
    IdleAuto(),
    ScoreAuto(),
    ScoreWithIntakeUpAuto(),
    IntakeAuto(),
    ScoreWhileIntakingAuto(),
    ClimbAuto(),
    Override(),
  }

  /** The addable states of superstructure */
  public enum AddableStates {
    Intaking(),
    Jostle(),
    IntakeUp(),
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
  public void setAddableState(AddableStates state) {
    m_addableState = state;
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
  public Command setAddableStateCommand(AddableStates state) {
    return Commands.runOnce(() -> setAddableState(state)).ignoringDisable(true);
  }

  /** Returns the current state of the superstructure. */
  @NotLogged
  public CurrentStates getCurrentState() {
    return m_currentState;
  }
}
