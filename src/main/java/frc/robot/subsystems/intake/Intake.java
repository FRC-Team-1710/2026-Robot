package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;
import frc.robot.constants.SubsystemConstants.IntakeConstants;
import java.util.function.BooleanSupplier;

@Logged
public class Intake {

  @NotLogged private final IntakeIO m_io;

  @NotLogged private boolean m_testing = false;

  @Logged(importance = Importance.CRITICAL)
  private IntakeStates m_currentState = IntakeStates.Up;

  @NotLogged private final BooleanSupplier m_bumpSupplier;

  @NotLogged private boolean m_brownoutMode = false;

  /**
   * Constructs an Intake subsystem.
   *
   * @param io the hardware/simulation IO implementation used to read sensors and command motors
   * @param bumpSupplier boolean to reverse the intake rollers when true (used for fixing jams or
   *     outaking quickly)
   */
  public Intake(IntakeIO io, BooleanSupplier bumpSupplier) {
    m_io = io;
    m_bumpSupplier = bumpSupplier;
  }

  public void toggleBrownout() {
    m_brownoutMode = !m_brownoutMode;
    Robot.telemetry().log("Brownout/Intake", m_brownoutMode);
  }

  /**
   * Sets the current intake state.
   *
   * @param state the intake state to set
   */
  public void setState(IntakeStates state) {
    if (m_testing) return;
    m_currentState = state;
    // m_io.setAngle( //TODO: dissables intake from going down
    //     IntakeStates.Down.setpoint,
    //     IntakeStates.Down.deploymentAcceleration,
    //     IntakeStates.Down.deploymentVelocity);
    m_io.setAngle(
        m_currentState.setpoint,
        m_currentState.deploymentAcceleration,
        m_currentState.deploymentVelocity);
    m_io.setIntakeVoltage(
        m_bumpSupplier.getAsBoolean()
            ? IntakeConstants.Rollers.Software.kReverseVoltage
            : (m_brownoutMode ? m_currentState.brownoutVoltage : m_currentState.voltage));
  }

  /**
   * Sets the current intake state for testing mode only.
   *
   * @param state the intake state to set
   */
  public void setStateTesting(IntakeStates state) {
    if (!m_testing) return;
    m_currentState = state;
    m_io.setAngle(
        m_currentState.setpoint,
        m_currentState.deploymentAcceleration,
        m_currentState.deploymentVelocity);
    m_io.setIntakeVoltage(
        m_bumpSupplier.getAsBoolean()
            ? IntakeConstants.Rollers.Software.kReverseVoltage
            : m_currentState.voltage);
  }

  /**
   * Enables or disables testing mode.
   *
   * @param testing true to enable testing mode
   */
  public void setTesting(boolean testing) {
    m_testing = testing;
  }

  /** Returns the current intake state. */
  @NotLogged
  public IntakeStates getState() {
    return m_currentState;
  }

  public enum IntakeStates {
    Up(
        IntakeConstants.Deployment.Hardware.kStowAngle,
        IntakeConstants.Deployment.Software.kDeployVelocity,
        IntakeConstants.Deployment.Software.kDeployAcceleration),
    Down(
        IntakeConstants.Deployment.Hardware.kDeployAngle,
        IntakeConstants.Deployment.Software.kDeployVelocity,
        IntakeConstants.Deployment.Software.kDeployAcceleration),
    Intake(
        IntakeConstants.Deployment.Hardware.kDeployAngle,
        IntakeConstants.Deployment.Software.kDeployVelocity,
        IntakeConstants.Deployment.Software.kDeployAcceleration,
        IntakeConstants.Rollers.Software.kRunVoltage,
        IntakeConstants.Rollers.Software.kBrownoutRunVoltage),
    UpAndIntake(
        IntakeConstants.Deployment.Hardware.kStowAngle,
        IntakeConstants.Deployment.Software.kShootingStowVelocity,
        IntakeConstants.Deployment.Software.kDeployAcceleration);
    // IntakeConstants.Rollers.Software.kRunVoltage,
    // IntakeConstants.Rollers.Software.kBrownoutRunVoltage);

    private final Angle setpoint;
    private final double deploymentVelocity;
    private final double deploymentAcceleration;
    private final Voltage voltage;
    private final Voltage brownoutVoltage;

    IntakeStates(Angle setpoint, double deploymentVelocity, double deploymentAcceleration) {
      this(setpoint, deploymentVelocity, deploymentAcceleration, Volts.of(0), Volts.of(0));
    }

    IntakeStates(
        Angle setpoint,
        double deploymentVelocity,
        double deploymentAcceleration,
        Voltage voltage,
        Voltage brownoutVoltage) {
      this.setpoint = setpoint;
      this.deploymentVelocity = deploymentVelocity;
      this.deploymentAcceleration = deploymentAcceleration;
      this.voltage = voltage;
      this.brownoutVoltage = brownoutVoltage;
    }
  }
}
