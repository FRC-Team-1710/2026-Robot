package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.constants.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.MatchState;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.SHOOTER_STATE;
import frc.robot.utils.shooterMath.ShooterMath;

@Logged
@SuppressWarnings("unused")
public class Superstructure {
  private CommandXboxController driver;
  private CommandXboxController mech;
  @NotLogged private CommandSwerveDrivetrain drivetrain;
  @NotLogged private Intake intake;
  @NotLogged private Shooter shooter;
  @NotLogged private Indexer indexer;

  private WantedStates wantedState = WantedStates.Default;
  private CurrentStates currentState = CurrentStates.Idle;

  private double shooterRPM;
  private double angle;

  private boolean shouldAssistLeft = false;

  public Superstructure(
      CommandXboxController driver,
      CommandXboxController mech,
      CommandSwerveDrivetrain drivetrain,
      Intake intake,
      Shooter shooter,
      Indexer indexer) {
    this.driver = driver;
    this.mech = mech;
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.shooter = shooter;
    this.indexer = indexer;
  }

  public void periodic() {
    currentState = handleStateTransitions();
    applyStates();
    applyRumble();

    shooterRPM = ShooterMath.calculateShootState().desiredRPM();
    angle = ShooterMath.calculateShootState().desiredAngle();

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
  private CurrentStates handleStateTransitions() {
    switch (wantedState) {
      case Default:
        return CurrentStates.Idle;
      case AssistRight:
        shouldAssistLeft = false;
        return CurrentStates.Assist;
      case AssistLeft:
        shouldAssistLeft = true;
        return CurrentStates.Assist;
      case Shoot:
        return drivetrain.inAllianceZone() ? CurrentStates.Score : CurrentStates.Shoot;
      case Intake:
        return CurrentStates.Intake;
      case IntakeAndShoot:
        return drivetrain.inAllianceZone()
            ? CurrentStates.ScoreWhileIntaking
            : CurrentStates.ShootWhileIntaking;
      case Climb:
        return CurrentStates.Climb;
      default:
        DriverStation.reportError("Switch from WantedState to CurrentState failed!", false);
        return CurrentStates.Idle;
    }
  }

  /** Applies the current states to the subsystems */
  private void applyStates() {
    switch (currentState) {
      case Idle:
        idle();
        break;
      case Assist:
        assist();
        break;
      case Shoot:
        shoot();
        break;
      case Score:
        score();
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
    }
  }

  private void idle() {
    drivetrain.setState(CommandSwerveDrivetrain.DriveStates.DRIVER_CONTROLLED);
    intake.setState(IntakeStates.Up);
    shooter.setState(SHOOTER_STATE.IDLE);
    indexer.setState(IndexStates.Idle);
  }

  private void assist() {
    drivetrain.setState(CommandSwerveDrivetrain.DriveStates.DRIVER_CONTROLLED);
    intake.setState(IntakeStates.Up);
    shooter.setState(SHOOTER_STATE.IDLE);
    indexer.setState(IndexStates.Idle);
  }

  private void shoot() {
    drivetrain.setRotationTarget(Alliance.redAlliance ? Rotation2d.kZero : Rotation2d.k180deg);
    drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);
    intake.setState(IntakeStates.Up);
    shooter.setState(SHOOTER_STATE.SHOOT);
    // TODO: Add auto shoot here
    indexer.setState(IndexStates.Idle);
  }

  private void score() {
    drivetrain.setRotationTarget(getRotationForScore());
    drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);
    intake.setState(IntakeStates.Up);
    shooter.setState(SHOOTER_STATE.SHOOT);
    // TODO: Add auto shoot here
    indexer.setState(IndexStates.Idle);
  }

  private void intake() {
    drivetrain.setState(CommandSwerveDrivetrain.DriveStates.DRIVER_CONTROLLED);
    intake.setState(IntakeStates.Intaking);
    shooter.setState(SHOOTER_STATE.IDLE);
    indexer.setState(IndexStates.Idle);
  }

  private void scoreWhileIntaking() {
    drivetrain.setRotationTarget(getRotationForScore());
    drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);
    intake.setState(IntakeStates.Intaking);
    shooter.setState(SHOOTER_STATE.SHOOT);
    // TODO: Add auto shoot here
    indexer.setState(IndexStates.Idle);
  }

  private void shootWhileIntaking() {
    drivetrain.setRotationTarget(Alliance.redAlliance ? Rotation2d.kZero : Rotation2d.k180deg);
    drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);
    intake.setState(IntakeStates.Intaking);
    shooter.setState(SHOOTER_STATE.SHOOT);
    // TODO: Add auto shoot here
    indexer.setState(IndexStates.Idle);
  }

  private void climb() {
    drivetrain.setState(CommandSwerveDrivetrain.DriveStates.DRIVER_CONTROLLED);
    intake.setState(IntakeStates.Up);
    shooter.setState(SHOOTER_STATE.IDLE);
    indexer.setState(IndexStates.Idle);
  }

  private Rotation2d getRotationForScore() {
    // TODO: Account for robot velocity for shooting on the move
    Translation2d targetPose =
        Alliance.redAlliance
            ? FieldConstants.kHubCenterBlue.rotateAround(
                new Translation2d(
                    FieldConstants.kFieldLength.div(2), FieldConstants.kFieldWidth.div(2)),
                Rotation2d.k180deg)
            : FieldConstants.kHubCenterBlue;
    return Rotation2d.fromRadians(
        Math.atan2(
            targetPose.getY() - drivetrain.getPose().getY(),
            targetPose.getX() - drivetrain.getPose().getX()));
  }

  /** The wanted states of superstructure */
  public enum WantedStates {
    Default(),
    AssistRight(),
    AssistLeft(),
    Shoot(),
    Intake(),
    IntakeAndShoot(),
    Climb(),
    DefaultAuto(),
  }

  /** The current states of superstructure */
  public enum CurrentStates {
    Idle(),
    Assist(),
    Shoot(),
    Score(),
    Intake(),
    ShootWhileIntaking(),
    ScoreWhileIntaking(),
    Climb()
  }

  /**
   * @param state the wanted state to set
   */
  public void setWantedState(WantedStates state) {
    wantedState = state;
  }

  /**
   * @param state the wanted state to set
   * @return a command that sets the wanted state
   */
  public Command setWantedStateCommand(WantedStates state) {
    return Commands.runOnce(() -> setWantedState(state));
  }
}
