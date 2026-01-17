package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;

@Logged
public class Superstructure {
  private WantedStates wantedState = WantedStates.Default;
  private CurrentStates currentState = CurrentStates.Idle;

  public Superstructure() {}

  public void periodic() {
    currentState = handleStateTransitions();
    applyStates();
  }

  private CurrentStates handleStateTransitions() {
    switch (wantedState) {
      case Default:
        return CurrentStates.Idle;
      case Targeting:
        return CurrentStates.Targeting;
      case Scoring:
        return CurrentStates.Scoring;
      case Intake:
        return CurrentStates.Intake;
      case ScoreWhileIntaking:
        return CurrentStates.ScoreWhileIntaking;
      case ShootWhileIntaking:
        return CurrentStates.ShootWhileIntaking;
      case Climb:
        return CurrentStates.Climb;
      default:
        DriverStation.reportError("Switch from WantedState to CurrentState failed!", false);
        return CurrentStates.Idle;
    }
  }

  private void applyStates() {
    switch (currentState) {
      case Idle:
        idle();
        break;
      case Targeting:
        targeting();
        break;
      case Scoring:
        scoring();
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

  private void idle() {}

  private void targeting() {}

  private void scoring() {}

  private void intake() {}

  private void scoreWhileIntaking() {}

  private void shootWhileIntaking() {}

  private void climb() {}

  public enum WantedStates {
    Default(),
    Targeting(),
    Scoring(),
    Intake(),
    ScoreWhileIntaking(),
    ShootWhileIntaking(),
    Climb()
  }

  public enum CurrentStates {
    Idle(),
    Targeting(),
    Scoring(),
    Intake(),
    ScoreWhileIntaking(),
    ShootWhileIntaking(),
    Climb()
  }
}
