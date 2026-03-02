package frc.robot.subsystems;

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
import frc.robot.constants.MatchState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.Feeder.FEEDER_STATE;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.SHOOTER_STATE;
import frc.robot.utils.shooterMath.ShooterMath2;

@Logged
public class Superstructure {
  @NotLogged private CommandXboxController driver;
  @NotLogged private CommandXboxController mech;
  @NotLogged private CommandSwerveDrivetrain drivetrain;
  @NotLogged private Intake intake;
  @NotLogged private Shooter shooter;
  @NotLogged private Indexer indexer;
  @NotLogged private Feeder feeder;

  @Logged(importance = Importance.CRITICAL)
  private WantedStates wantedState = WantedStates.Default;

  @Logged(importance = Importance.CRITICAL)
  private CurrentStates currentState = CurrentStates.Idle;

  @Logged(importance = Importance.INFO)
  private boolean didIntake = false;

  @Logged(importance = Importance.CRITICAL)
  private AddableStates addableState = AddableStates.Jostle;

  public Superstructure(
      CommandXboxController driver,
      CommandXboxController mech,
      CommandSwerveDrivetrain drivetrain,
      Intake intake,
      Shooter shooter,
      Indexer indexer,
      Feeder feeder) {
    this.driver = driver;
    this.mech = mech;
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.shooter = shooter;
    this.indexer = indexer;
    this.feeder = feeder;
  }

  public void periodic() {
    shooter.setGoingTowardsAllianceZone(drivetrain.isGoingTowardsAllianceZone());
    shooter.setDidIntake(didIntake);

    currentState = handleStateTransitions();
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

  public void applyRumble() {
    var timeUntilRumble = MatchState.timeTillActive().plus(MatchState.timeTillInactive());
    if (!DriverStation.isAutonomous() && MatchState.autonomousWinnerIsRed.isPresent()) {
      if (!timeUntilRumble.isEquivalent(Seconds.of(0))) {
        mech.setRumble(RumbleType.kBothRumble, timeUntilRumble.in(Seconds) < 2.5 ? 1 : 0);
      } else {
        mech.setRumble(RumbleType.kBothRumble, 0);
      }
    }
  }

  /**
   * Handles the transitions from wanted states to current states
   *
   * @return the new current state
   */
  @NotLogged
  private CurrentStates handleStateTransitions() {
    return switch (wantedState) {
      case Default -> CurrentStates.Idle;
      case Shoot ->
          switch (addableState) {
            case Jostle -> CurrentStates.Score;
            case IntakeUp -> CurrentStates.ScoreWithIntakeUp;
          };
      case Intake -> CurrentStates.Intake;
      case IntakeAndShoot -> CurrentStates.ScoreWhileIntaking;
      case Climb -> CurrentStates.Climb;
      case DefaultAuto -> CurrentStates.IdleAuto;
      case ShootAuto ->
          switch (addableState) {
            case Jostle -> CurrentStates.ScoreAuto;
            case IntakeUp -> CurrentStates.ScoreWithIntakeUpAuto;
          };
      case IntakeAuto -> CurrentStates.IntakeAuto;
      case IntakeAndShootAuto -> CurrentStates.ScoreWhileIntakingAuto;
      case ClimbAuto -> CurrentStates.ClimbAuto;
    };
  }

  /** Applies the current states to the subsystems */
  private void applyStates() {
    switch (currentState) {
      case Idle:
        idle();
        break;
      case Score:
        score();
        break;
      case ScoreWithIntakeUp:
        scoreWithIntakeUp();
        break;
      case Intake:
        intake();
        break;
      case ScoreWhileIntaking:
        scoreWhileIntaking();
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
    }
  }

  private void idle() {
    drivetrain.setState(CommandSwerveDrivetrain.DriveStates.DRIVER_CONTROLLED);
    shooter.setState(SHOOTER_STATE.IDLE);
    indexer.setState(IndexStates.Idle);
    feeder.setState(FEEDER_STATE.STOP);
  }

  private void score() {
    drivetrain.setRotationTarget(getRotationForScore());
    drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);
    intake.setState(IntakeStates.Jostle);
    shooter.setState(SHOOTER_STATE.SHOOT);
    indexer.setState(anyAtTarget() ? IndexStates.Indexing : IndexStates.Idle);
    feeder.setState(
        allAtTarget()
            ? FEEDER_STATE.FEEDING
            : leftAtTarget()
                ? FEEDER_STATE.FEEDING_LEFT
                : rightAtTarget() ? FEEDER_STATE.FEEDING_RIGHT : FEEDER_STATE.STOP);

    didIntake = false;
  }

  private void scoreWithIntakeUp() {
    drivetrain.setRotationTarget(getRotationForScore());
    drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);
    intake.setState(IntakeStates.Up);
    shooter.setState(SHOOTER_STATE.SHOOT);
    indexer.setState(anyAtTarget() ? IndexStates.Indexing : IndexStates.Idle);
    feeder.setState(
        allAtTarget()
            ? FEEDER_STATE.FEEDING
            : leftAtTarget()
                ? FEEDER_STATE.FEEDING_LEFT
                : rightAtTarget() ? FEEDER_STATE.FEEDING_RIGHT : FEEDER_STATE.STOP);

    didIntake = false;
  }

  private void intake() {
    drivetrain.setState(CommandSwerveDrivetrain.DriveStates.DRIVER_CONTROLLED);
    intake.setState(IntakeStates.Intaking);
    shooter.setState(SHOOTER_STATE.IDLE);
    indexer.setState(IndexStates.Idle);
    feeder.setState(FEEDER_STATE.STOP);

    didIntake = true;
  }

  private void scoreWhileIntaking() {
    drivetrain.setRotationTarget(getRotationForScore());
    drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);
    intake.setState(IntakeStates.Intaking);
    shooter.setState(SHOOTER_STATE.SHOOT);
    indexer.setState(anyAtTarget() ? IndexStates.Indexing : IndexStates.Idle);
    feeder.setState(
        allAtTarget()
            ? FEEDER_STATE.FEEDING
            : leftAtTarget()
                ? FEEDER_STATE.FEEDING_LEFT
                : rightAtTarget() ? FEEDER_STATE.FEEDING_RIGHT : FEEDER_STATE.STOP);

    didIntake = false;
  }

  private void climb() {
    drivetrain.setState(CommandSwerveDrivetrain.DriveStates.DRIVER_CONTROLLED);
    shooter.setState(SHOOTER_STATE.IDLE);
    indexer.setState(IndexStates.Idle);
    feeder.setState(FEEDER_STATE.STOP);
  }

  private void idleAuto() {
    feeder.setState(FEEDER_STATE.STOP);
    shooter.setState(SHOOTER_STATE.IDLE);
    indexer.setState(IndexStates.Idle);
  }

  private void scoreAuto() {
    intake.setState(IntakeStates.Jostle);
    shooter.setState(SHOOTER_STATE.SHOOT);
    indexer.setState(anyAtTarget() ? IndexStates.Indexing : IndexStates.Idle);
    feeder.setState(
        allAtTarget()
            ? FEEDER_STATE.FEEDING
            : leftAtTarget()
                ? FEEDER_STATE.FEEDING_LEFT
                : rightAtTarget() ? FEEDER_STATE.FEEDING_RIGHT : FEEDER_STATE.STOP);

    didIntake = false;
  }

  private void scoreWithIntakeUpAuto() {
    intake.setState(IntakeStates.Up);
    shooter.setState(SHOOTER_STATE.SHOOT);
    indexer.setState(anyAtTarget() ? IndexStates.Indexing : IndexStates.Idle);
    feeder.setState(
        allAtTarget()
            ? FEEDER_STATE.FEEDING
            : leftAtTarget()
                ? FEEDER_STATE.FEEDING_LEFT
                : rightAtTarget() ? FEEDER_STATE.FEEDING_RIGHT : FEEDER_STATE.STOP);

    didIntake = false;
  }

  private void intakeAuto() {
    intake.setState(IntakeStates.Intaking);
    shooter.setState(SHOOTER_STATE.IDLE);
    indexer.setState(IndexStates.Idle);
    feeder.setState(FEEDER_STATE.STOP);

    didIntake = true;
  }

  private void scoreWhileIntakingAuto() {
    intake.setState(IntakeStates.Intaking);
    shooter.setState(SHOOTER_STATE.SHOOT);
    indexer.setState(anyAtTarget() ? IndexStates.Indexing : IndexStates.Idle);
    feeder.setState(
        allAtTarget()
            ? FEEDER_STATE.FEEDING
            : leftAtTarget()
                ? FEEDER_STATE.FEEDING_LEFT
                : rightAtTarget() ? FEEDER_STATE.FEEDING_RIGHT : FEEDER_STATE.STOP);

    didIntake = false;
  }

  private void climbAuto() {
    shooter.setState(SHOOTER_STATE.IDLE);
    indexer.setState(IndexStates.Idle);
    feeder.setState(FEEDER_STATE.STOP);
  }

  @Logged(importance = Importance.INFO)
  public boolean allAtTarget() {
    return leftAtTarget() && rightAtTarget();
  }

  @NotLogged
  public boolean anyAtTarget() {
    return leftAtTarget() || rightAtTarget();
  }

  @Logged(importance = Importance.CRITICAL)
  public boolean leftAtTarget() {
    return shooter.isAtLeftTargetVelocity()
        && shooter.isHoodAtLeftTargetAngle()
        && ShooterMath2.currentSolution.shooterLeft().inTolerance(drivetrain.getRotation());
  }

  @Logged(importance = Importance.CRITICAL)
  public boolean rightAtTarget() {
    return shooter.isAtRightTargetVelocity()
        && shooter.isHoodAtRightTargetAngle()
        && ShooterMath2.currentSolution.shooterRight().inTolerance(drivetrain.getRotation());
  }

  @NotLogged
  private Rotation2d getRotationForScore() {
    return ShooterMath2.currentSolution.robotHeading().plus(Rotation2d.k180deg);
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
  }

  /** The current states of superstructure */
  public enum CurrentStates {
    Idle(),
    Score(),
    ScoreWithIntakeUp(),
    Intake(),
    ScoreWhileIntaking(),
    Climb(),
    IdleAuto(),
    ScoreAuto(),
    ScoreWithIntakeUpAuto(),
    IntakeAuto(),
    ScoreWhileIntakingAuto(),
    ClimbAuto(),
  }

  public enum AddableStates {
    Jostle(),
    IntakeUp(),
  }

  /**
   * @param state the wanted state to set
   */
  public void setWantedState(WantedStates state) {
    wantedState = state;
  }

  /**
   * @param state the addable state to set
   */
  public void setAddableState(AddableStates state) {
    addableState = state;
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

  @NotLogged
  public CurrentStates getCurrentState() {
    return currentState;
  }
}
