package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.MechanismUtil;
import frc.robot.utils.TalonFXUtil;

/**
 * Simulation implementation of the flywheel subsystem.
 *
 * <p>This class simulates a flywheel mechanism (rotating mass for shooting projectiles) and
 * provides visual feedback through SmartDashboard. It uses WPILib's FlywheelSim for physics
 * simulation and Mechanism2d for visualization.
 */
public class FlywheelSIM extends Flywheel {

  // ==================== Physical Constants ====================

  /** Gear ratio between motor and flywheel (motor rotations : flywheel rotations) */
  private static final double GEAR_RATIO = 1.0;

  /** Moment of inertia of the flywheel in kg⋅m² */
  private static final double FLYWHEEL_MOI = 0.01;

  /** Simulation update period in seconds (20ms = standard robot loop) */
  private static final double SIM_PERIOD_SECONDS = 0.020;

  /** Visual radius of the flywheel in pixels */
  private static final double FLYWHEEL_RADIUS = 80.0;

  // ==================== Simulation Components ====================

  /** DC motor model (Kraken X60) - using 2 motors as per hardware config */
  private final DCMotor dcMotor = DCMotor.getKrakenX60(2);

  /** Physics simulation of the flywheel mechanism */
  private final DCMotorSim flywheelSim;

  /** Mechanism visualization helper */
  private final MechanismUtil.FlywheelMechanism flywheelMechanism;

  /**
   * Constructs a new FlywheelSIM instance.
   *
   * <p>Initializes the physics simulation and creates the visual representation of the flywheel
   * mechanism on SmartDashboard.
   */
  public FlywheelSIM() {
    super();

    // Configure gear ratio for simulation (direct drive, but set for consistency)
    config.Feedback.RotorToSensorRatio = GEAR_RATIO;
    config.Slot0.kS = 0.0; // Static gain (feedforward)
    config.Slot0.kV = 0.12; // Velocity gain (12V / 100 RPS ≈ 0.12)
    config.Slot0.kP = 0.1; // Proportional gain (tune this!)
    config.MotionMagic.MotionMagicCruiseVelocity = 100.0; // Max velocity (RPS)
    config.MotionMagic.MotionMagicAcceleration = 400.0; // Max acceleration (RPS²)
    TalonFXUtil.applyConfigWithRetries(leader, config);

    LinearSystem<N2, N1, N2> linearSystem =
        LinearSystemId.createDCMotorSystem(
            dcMotor, FLYWHEEL_MOI, GEAR_RATIO); // Direct drive (1:1 ratio)
    // Initialize the physics simulation (no gravity for flywheels)
    flywheelSim = new DCMotorSim(linearSystem, dcMotor);

    // Create the mechanism visualization
    flywheelMechanism = new MechanismUtil.FlywheelMechanism("Flywheel", FLYWHEEL_RADIUS);

    // Publish the mechanism visualization to SmartDashboard
    SmartDashboard.putData("Flywheel Sim", flywheelMechanism.getMechanism());
  }

  /**
   * Updates the flywheel simulation each periodic cycle.
   *
   * <p>This method performs the following tasks:
   *
   * <ul>
   *   <li>Feeds the motor voltage into the physics simulation
   *   <li>Steps the simulation forward by one period
   *   <li>Simulates battery voltage sag from current draw
   *   <li>Updates the motor encoder simulation values
   *   <li>Animates the visual mechanism display
   *   <li>Publishes telemetry data to SmartDashboard
   * </ul>
   */
  @Override
  public void simulationPeriodic() {
    // Feed the motor voltage from the controller into the physics simulation
    flywheelSim.setInput(leader.getMotorVoltage().getValueAsDouble());

    // Step the simulation forward by one robot loop period
    flywheelSim.update(SIM_PERIOD_SECONDS);

    // Simulate battery voltage sag based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));

    // Get current flywheel velocity in radians per second
    double velocityRadPerSec = flywheelSim.getAngularVelocityRadPerSec();

    // Convert flywheel velocity to motor velocity (accounting for gear ratio)
    double motorVelocity =
        RotationsPerSecond.of((velocityRadPerSec / (2 * Math.PI)) * GEAR_RATIO)
            .in(RotationsPerSecond);

    // Update the simulated motor encoder velocity
    leader.getSimState().setRotorVelocity(motorVelocity);

    // Calculate motor position by integrating velocity over time
    double currentPosition = leader.getRotorPosition().getValueAsDouble();
    double newPosition = currentPosition + (motorVelocity * SIM_PERIOD_SECONDS);
    leader.getSimState().setRawRotorPosition(newPosition);

    // Animate the visual representation
    updateVisualization(velocityRadPerSec);

    // Publish sim-specific telemetry (other values are auto-logged from base class)
    SmartDashboard.putNumber("Flywheel Sim Current (A)", flywheelSim.getCurrentDrawAmps());
  }

  /**
   * Updates the visual representation of the flywheel.
   *
   * <p>Delegates to the FlywheelMechanism utility to update the spokes rotation and color based on
   * current velocity and state.
   *
   * @param velocityRadPerSec The current angular velocity in radians per second
   */
  private void updateVisualization(double velocityRadPerSec) {
    // Update the mechanism visualization
    flywheelMechanism.update(velocityRadPerSec, SIM_PERIOD_SECONDS, isAtTarget());
  }
}
