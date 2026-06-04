package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
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
import frc.robot.utils.shooterMath.ShooterMath4;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  private final CommandXboxController m_driver;

  private CommandSwerveDrivetrain m_drivetrain;
  private Intake m_intake;
  private Shooter m_shooter;
  private Indexer m_indexer;
  private Feeder m_feeder;

  private WantedStates m_wantedState = WantedStates.Default;

  private CurrentStates m_currentState = CurrentStates.Idle;

  private IntakeAddableStates m_intakeAddableState = IntakeAddableStates.Intaking;

  private final Debouncer m_debouncerDrive = new Debouncer(0.5);

  private boolean m_wasAtTarget = false;

  private double m_restingVoltage = 13.0;

  /**
   * Constructs the superstructure with all subsystem references.
   *
   * @param driver the driver controller
   * @param m_drivetrain the swerve m_drivetrain
   * @param m_intake the m_intake subsystem
   * @param m_shooter the m_shooter subsystem
   * @param m_indexer the m_indexer subsystem
   * @param m_feeder the m_feeder subsystem
   */
  public Superstructure(
      CommandXboxController driver,
      CommandSwerveDrivetrain drivetrain,
      Intake intake,
      Shooter shooter,
      Indexer indexer,
      Feeder feeder) {
    this.m_driver = driver;
    this.m_drivetrain = drivetrain;
    this.m_intake = intake;
    this.m_shooter = shooter;
    this.m_indexer = indexer;
    this.m_feeder = feeder;
  }

  /** Runs periodic logic for state transitions and subsystem coordination. */
  public void periodic() {
    m_currentState = handleStateTransitions();
    applyStates();

    applyRumble();

    Logger.recordOutput("Misc/RedAlliance", Alliance.redAlliance);

    Logger.recordOutput(
        "Timing/TimeTillActive", Math.round(MatchState.timeTillActive().in(Seconds) * 10.0) / 10.0);
    Logger.recordOutput(
        "Timing/TimeTillInactive",
        Math.round(MatchState.timeTillInactive().in(Seconds) * 10.0) / 10.0);
    Logger.recordOutput("Timing/IsActive", MatchState.isActive());

    Logger.recordOutput(
        "Misc/AutonomousWinnerIsRed",
        MatchState.autonomousWinnerIsRed.isPresent()
            ? String.valueOf(MatchState.autonomousWinnerIsRed.get())
            : "No data");

    Logger.recordOutput("Superstructure/WantedState", m_wantedState);
    Logger.recordOutput("Superstructure/CurrentState", m_currentState);
    Logger.recordOutput("Superstructure/IntakeAddableState", m_intakeAddableState);
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

  /** Sets the resting voltage used for driver rumble. */
  public void setRestingVoltage(double voltage) {
    m_restingVoltage = voltage;
  }

  /** Returns whether the current state does not use the m_intake. */
  public boolean currentStateDoesntUseIntake() {
    return !currentStateUsesIntake();
  }

  /** Applies rumble feedback to the mechanism controller based on voltage. */
  public void applyRumble() {
    if (DriverStation.isTeleopEnabled()) {
      double rumble =
          1
              - ((RobotController.getBatteryVoltage() - RobotController.getBrownoutVoltage())
                  / (m_restingVoltage - RobotController.getBrownoutVoltage()));
      m_driver.setRumble(RumbleType.kBothRumble, rumble);
      Logger.recordOutput("Superstructure/Rumble", rumble);
    } else {
      m_driver.setRumble(RumbleType.kBothRumble, 0);
      Logger.recordOutput("Superstructure/Rumble", 0);
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
                          > FieldConstants.kBumpDistanceFromDS.in(Meters) - Units.inchesToMeters(6))
                  || (Alliance.redAlliance
                      && m_drivetrain.getPose().getX()
                          < FieldConstants.kFieldLength
                                  .minus(FieldConstants.kBumpDistanceFromDS)
                                  .in(Meters)
                              + Units.inchesToMeters(6)))
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
                          > FieldConstants.kBumpDistanceFromDS.in(Meters) - Units.inchesToMeters(6))
                  || (Alliance.redAlliance
                      && m_drivetrain.getPose().getX()
                          < FieldConstants.kFieldLength
                                  .minus(FieldConstants.kBumpDistanceFromDS)
                                  .in(Meters)
                              + Units.inchesToMeters(6)))
              ? CurrentStates.ShootWhileIntaking
              : CurrentStates.ScoreWhileIntaking;
      case DefaultAuto -> CurrentStates.IdleAuto;
      case ShootAuto ->
          switch (m_intakeAddableState) {
            case Jostle -> CurrentStates.ScoreAuto;
            case IntakeUp -> CurrentStates.ScoreWithIntakeUpAuto;
            case Intaking -> CurrentStates.ScoreWhileIntakingAuto;
          };
      case IntakeAuto -> CurrentStates.IntakeAuto;
      case IntakeAndShootAuto -> CurrentStates.ScoreWhileIntakingAuto;
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

    m_wasAtTarget = false;
  }

  private void score() {
    m_drivetrain.setRotationTarget(getRotationForScore());
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);

    if (!m_wasAtTarget) {
      m_wasAtTarget = flywheelAtTarget();
    }

    m_intake.setState(IntakeStates.Jostle);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_feeder.setState(m_wasAtTarget ? FEEDER_STATE.FEEDING : FEEDER_STATE.STOP);
    m_indexer.setState(m_wasAtTarget ? IndexStates.Indexing : IndexStates.Idle);
  }

  private void scoreWithIntakeUp() {
    m_drivetrain.setRotationTarget(getRotationForScore());
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);

    if (!m_wasAtTarget) {
      m_wasAtTarget = flywheelAtTarget();
    }

    m_intake.setState(IntakeStates.UpAndIntake);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_feeder.setState(m_wasAtTarget ? FEEDER_STATE.FEEDING : FEEDER_STATE.STOP);
    m_indexer.setState(m_wasAtTarget ? IndexStates.Indexing : IndexStates.Idle);
  }

  private void shoot() {
    m_drivetrain.setRotationTarget(getRotationForShoot());
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);

    if (!m_wasAtTarget) {
      m_wasAtTarget = flywheelAtTarget();
    }

    m_intake.setState(IntakeStates.Jostle);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_feeder.setState(m_wasAtTarget ? FEEDER_STATE.FEEDING : FEEDER_STATE.STOP);
    m_indexer.setState(m_wasAtTarget ? IndexStates.Indexing : IndexStates.Idle);
  }

  private void shootWithIntakeUp() {
    m_drivetrain.setRotationTarget(getRotationForShoot());
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);

    if (!m_wasAtTarget) {
      m_wasAtTarget = flywheelAtTarget();
    }

    m_intake.setState(IntakeStates.UpAndIntake);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_feeder.setState(m_wasAtTarget ? FEEDER_STATE.FEEDING : FEEDER_STATE.STOP);
    m_indexer.setState(m_wasAtTarget ? IndexStates.Indexing : IndexStates.Idle);
  }

  private void intake() {
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.DRIVER_CONTROLLED);
    m_intake.setState(IntakeStates.Intaking);
    m_shooter.setState(SHOOTER_STATE.IDLE);
    m_indexer.setState(IndexStates.Idle);
    m_feeder.setState(FEEDER_STATE.STOP);

    m_wasAtTarget = false;
  }

  private void scoreWhileIntaking() {
    m_drivetrain.setRotationTarget(getRotationForScore());
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);

    if (!m_wasAtTarget) {
      m_wasAtTarget = flywheelAtTarget();
    }

    m_intake.setState(IntakeStates.Intaking);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_feeder.setState(m_wasAtTarget ? FEEDER_STATE.FEEDING : FEEDER_STATE.STOP);
    m_indexer.setState(m_wasAtTarget ? IndexStates.Indexing : IndexStates.Idle);
  }

  private void shootWhileIntaking() {
    m_drivetrain.setRotationTarget(getRotationForShoot());
    m_drivetrain.setState(CommandSwerveDrivetrain.DriveStates.ROTATION_LOCK);

    if (!m_wasAtTarget) {
      m_wasAtTarget = flywheelAtTarget();
    }

    m_intake.setState(IntakeStates.Intaking);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_feeder.setState(m_wasAtTarget ? FEEDER_STATE.FEEDING : FEEDER_STATE.STOP);
    m_indexer.setState(m_wasAtTarget ? IndexStates.Indexing : IndexStates.Idle);
  }

  private void idleAuto() {
    m_feeder.setState(FEEDER_STATE.STOP);
    m_indexer.setState(IndexStates.Idle);
    m_shooter.setState(SHOOTER_STATE.IDLE);
  }

  private void scoreAuto() {
    m_intake.setState(IntakeStates.Jostle);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_feeder.setState(flywheelAtTarget() ? FEEDER_STATE.FEEDING : FEEDER_STATE.STOP);
    m_indexer.setState(flywheelAtTarget() ? IndexStates.Indexing : IndexStates.Idle);
  }

  private void scoreWithIntakeUpAuto() {
    m_intake.setState(IntakeStates.UpAndIntake);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_feeder.setState(flywheelAtTarget() ? FEEDER_STATE.FEEDING : FEEDER_STATE.STOP);
    m_indexer.setState(flywheelAtTarget() ? IndexStates.Indexing : IndexStates.Idle);
  }

  private void intakeAuto() {
    m_intake.setState(IntakeStates.IntakingAuto);
    m_shooter.setState(SHOOTER_STATE.IDLE);
    m_indexer.setState(IndexStates.Idle);
    m_feeder.setState(FEEDER_STATE.STOP);
  }

  private void scoreWhileIntakingAuto() {
    m_intake.setState(IntakeStates.Intaking);
    m_shooter.setState(SHOOTER_STATE.SHOOT);
    m_feeder.setState(flywheelAtTarget() ? FEEDER_STATE.FEEDING : FEEDER_STATE.STOP);
    m_indexer.setState(flywheelAtTarget() ? IndexStates.Indexing : IndexStates.Idle);
  }

  private void override() {
    m_intake.setState(
        m_intakeAddableState == IntakeAddableStates.IntakeUp
            ? IntakeStates.UpAndIntake
            : m_intakeAddableState == IntakeAddableStates.Intaking
                ? IntakeStates.Intaking
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

  @AutoLogOutput(key = "Superstructure/DriveAtTarget")
  public boolean driveAtTarget() {
    return m_debouncerDrive.calculate(
        Math.abs(m_drivetrain.getRotation().minus(m_drivetrain.getRotationTarget()).getDegrees())
            <= 10);
  }

  public boolean flywheelAtTarget() {
    return m_shooter.isAtTargetVelocity() && m_shooter.isHoodAtTargetAngle() && driveAtTarget();
  }

  public Rotation2d getRotationForScore() {
    return ShooterMath4.currentSolution.robotHeading().plus(Rotation2d.k180deg);
  }

  public Rotation2d getRotationForShoot() {
    return ShooterMath4.currentPassingSolution.robotHeading().plus(Rotation2d.k180deg);
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
    Default(),
    Shoot(),
    Intake(),
    IntakeAndShoot(),
    DefaultAuto(),
    ShootAuto(),
    IntakeAuto(),
    IntakeAndShootAuto(),
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
    IdleAuto(),
    ScoreAuto(),
    ScoreWithIntakeUpAuto(),
    IntakeAuto(),
    ScoreWhileIntakingAuto(),
    Override()
  }

  /** The addable states of intake */
  public enum IntakeAddableStates {
    Intaking(),
    Jostle(),
    IntakeUp()
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

  /** Returns the current state of the superstructure. */
  public CurrentStates getCurrentState() {
    return m_currentState;
  }
}
