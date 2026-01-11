package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.utils.MechanismUtil;
import frc.robot.utils.TalonFXUtil;

/**
 * Simulation implementation of the arm subsystem.
 *
 * <p>This class simulates a single-jointed arm mechanism with gravity and provides visual feedback
 * through SmartDashboard. It uses WPILib's SingleJointedArmSim for physics simulation and
 * Mechanism2d for visualization.
 */
public class ArmSIM extends Arm {

  // ==================== Physical Constants ====================

  /** Gear ratio between motor and arm (motor rotations : arm rotations) */
  private static final double GEAR_RATIO = 25.0;

  /** Length of the arm in meters */
  private static final double ARM_LENGTH = 1.0;

  /** Mass of the arm in kilograms */
  private static final double ARM_MASS_KG = 5.0;

  /** Minimum angle of the arm in radians (horizontal position) */
  private static final double MIN_ANGLE_RAD = Units.rotationsToRadians(-1);

  /** Maximum angle of the arm in radians (vertical position, 90 degrees) */
  private static final double MAX_ANGLE_RAD = Units.rotationsToRadians(1);

  /** Starting angle of the arm in radians */
  private static final double STARTING_ANGLE_RAD = 0.0;

  /** Simulation update period in seconds (20ms = standard robot loop) */
  private static final double SIM_PERIOD_SECONDS = 0.020;

  /** Target visual length for the arm in pixels */
  private static final double ARM_VISUAL_LENGTH = 200.0;

  // ==================== Simulation Components ====================

  /** DC motor model (Kraken X60) */
  private final DCMotor dcMotor = DCMotor.getKrakenX60(1);

  /** Physics simulation of the arm mechanism */
  private final SingleJointedArmSim armSim;

  /** Mechanism visualization helper */
  private final MechanismUtil.ArmMechanism armMechanism;

  /**
   * Constructs a new ArmSIM instance.
   *
   * <p>Initializes the physics simulation and creates the visual representation of the arm
   * mechanism on SmartDashboard.
   */
  public ArmSIM() {
    super();

    // Configure gear ratio for simulation (tells Phoenix how motor relates to sensor)
    // For every 25 motor rotations, the CANcoder sees 1 rotation
    config.Feedback.RotorToSensorRatio = GEAR_RATIO;
    config.Slot0.kG = 0.0; // Gravity gain
    config.Slot0.kS = 0.0; // Static gain
    config.Slot0.kP = 160; // Proportional gain
    config.Slot0.kD = 30; // Derivative gain
    config.MotionMagic.MotionMagicCruiseVelocity = 1; // Max velocity (RPS)
    config.MotionMagic.MotionMagicAcceleration = 4; // Max acceleration (RPS²)
    TalonFXUtil.applyConfigWithRetries(leader, config);

    // Initialize the physics simulation
    armSim =
        new SingleJointedArmSim(
            dcMotor,
            GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(ARM_LENGTH, ARM_MASS_KG),
            ARM_LENGTH,
            MIN_ANGLE_RAD,
            MAX_ANGLE_RAD,
            false, // Simulate gravity
            STARTING_ANGLE_RAD);

    // Create the mechanism visualization
    armMechanism = new MechanismUtil.ArmMechanism("Arm", ARM_VISUAL_LENGTH);

    // Publish the mechanism visualization to SmartDashboard
    SmartDashboard.putData("Arm Sim", armMechanism.getMechanism());
  }

  /**
   * Updates the arm simulation each periodic cycle.
   *
   * <p>This method performs the following tasks:
   *
   * <ul>
   *   <li>Feeds the motor voltage into the physics simulation
   *   <li>Steps the simulation forward by one period
   *   <li>Simulates battery voltage sag from current draw
   *   <li>Updates the motor encoder simulation values
   *   <li>Updates the visual mechanism display
   *   <li>Publishes telemetry data to SmartDashboard
   * </ul>
   */
  @Override
  public void simulationPeriodic() {
    // Feed the motor voltage from the controller into the physics simulation
    armSim.setInput(leader.getMotorVoltage().getValueAsDouble());

    // Step the simulation forward by one robot loop period
    armSim.update(SIM_PERIOD_SECONDS);

    // Simulate battery voltage sag based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

    // Convert arm angle to encoder rotations (encoder is on the arm, not the motor)
    double encoderPosition = Radians.of(armSim.getAngleRads()).in(Rotations);
    double encoderVelocity =
        RadiansPerSecond.of(armSim.getVelocityRadPerSec()).in(RotationsPerSecond);

    // Update the CANcoder simulation (this is what the base class reads from)
    encoder.getSimState().setRawPosition(encoderPosition);
    encoder.getSimState().setVelocity(encoderVelocity);

    // Also update motor sim for completeness (motor rotations = encoder * gear ratio)
    double motorPosition = encoderPosition * GEAR_RATIO;
    double motorVelocity = encoderVelocity * GEAR_RATIO;
    leader.getSimState().setRawRotorPosition(motorPosition);
    leader.getSimState().setRotorVelocity(motorVelocity);

    // Update the visual representation
    updateVisualization();

    // Publish sim-specific telemetry (other values are auto-logged from base class)
    Robot.telemetry().log("Arm Sim Current (A)", armSim.getCurrentDrawAmps());
  }

  /**
   * Updates the visual representation of the arm.
   *
   * <p>Delegates to the ArmMechanism utility to update the arm angle and color based on current
   * state.
   */
  private void updateVisualization() {
    // Get current position from the base class method (reads from simulated encoder)
    double currentAngleDeg = getPosition().in(Degrees);

    // Update the mechanism visualization
    armMechanism.update(currentAngleDeg, isAtTarget());
  }
}