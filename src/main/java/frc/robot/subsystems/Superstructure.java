package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.MatchState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.utils.Log;

@Logged
public class Superstructure {
  private CommandXboxController driver;
  private CommandXboxController mech;
  private CommandSwerveDrivetrain drivetrain;
  private Intake intake;

  private WantedStates wantedState = WantedStates.Default;
  private CurrentStates currentState = CurrentStates.Idle;

  public Superstructure(
      CommandXboxController driver,
      CommandXboxController mech,
      CommandSwerveDrivetrain drivetrain,
      Intake intake) {
    this.driver = driver;
    this.mech = mech;
    this.drivetrain = drivetrain;
    this.intake = intake;
  }

  public void periodic() {
    currentState = handleStateTransitions();
    applyStates();
    applyRumble();
  }

  public void applyRumble() {
    var timeUntilRumble = MatchState.timeTillActive().plus(MatchState.timeTillInactive());
    if (!DriverStation.isAutonomous() && MatchState.autonomousWinnerIsRed.isPresent()) {
      Log.log("Num1", MatchState.timeTillActive().in(Seconds));
      Log.log("Num2", MatchState.timeTillInactive().in(Seconds));
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
    intake.setState(IntakeStates.Up);
  }

  private void shoot() {
    intake.setState(IntakeStates.Up);
  }

  private void score() {
    intake.setState(IntakeStates.Up);
  }

  private void intake() {
    intake.setState(IntakeStates.Intaking);
  }

  private void scoreWhileIntaking() {
    intake.setState(IntakeStates.Intaking);
  }

  private void shootWhileIntaking() {
    intake.setState(IntakeStates.Intaking);
  }

  private void climb() {
    intake.setState(IntakeStates.Up);
  }

  /** The wanted states of superstructure */
  public enum WantedStates {
    Default(),
    Shoot(),
    Intake(),
    IntakeAndShoot(),
    Climb()
  }

  /** The current states of superstructure */
  public enum CurrentStates {
    Idle(),
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
