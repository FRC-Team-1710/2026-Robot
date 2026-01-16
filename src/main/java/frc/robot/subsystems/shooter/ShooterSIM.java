package frc.robot.subsystems.shooter;

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

public class ShooterSIM extends Shooter {
  
  // ==================== Physical Constants ====================

  private static final double GEAR_RATIO = 1.0;

  /** Moment of inertia of the Shooter in kg⋅m² */
  private static final double Shooter_MOI = 0.01;

  private static final double SIM_PERIOD_SECONDS = 0.020; // 20ms

  /** Visual radius of the Shooter in pixels */
  private static final double Shooter_RADIUS = 80.0;

  // ==================== Simulation Components ====================

  // TODO : Replace motor type (if nessecary).
  private final DCMotor m_dcMotor = DCMotor.getKrakenX60(2);

  private final DCMotorSim m_shooterSim;

  private final MechanismUtil.FlywheelMechanism ShooterMechanism;

  public ShooterSIM() {
    super();

    // Configure gear ratio for simulation (direct drive, but set for consistency)
    this.m_config.Feedback.RotorToSensorRatio = GEAR_RATIO;
    this.m_config.Slot0.kS = 0.0; // Static gain (feedforward)
    this.m_config.Slot0.kV = 0.12; // Velocity gain (12V / 100 RPS ≈ 0.12)
    this.m_config.Slot0.kP = 0.1; // Proportional gain (tune this!)
    this.m_config.MotionMagic.MotionMagicCruiseVelocity = 100.0; // Max velocity (RPS)
    this.m_config.MotionMagic.MotionMagicAcceleration = 400.0; // Max acceleration (RPS²)
    TalonFXUtil.applyConfigWithRetries(m_flyWheen, this.m_config);

    LinearSystem<N2, N1, N2> linearSystem = LinearSystemId.createDCMotorSystem(this.m_dcMotor, Shooter_MOI, GEAR_RATIO); // Direct drive (1:1 ratio)
    
    this.m_shooterSim = new DCMotorSim(linearSystem, this.m_dcMotor);

    ShooterMechanism = new MechanismUtil.FlywheelMechanism("Shooter", Shooter_RADIUS);

    SmartDashboard.putData("Shooter Sim", ShooterMechanism.getMechanism());
  }

  @Override
  public void simulationPeriodic() {
    this.m_shooterSim.setInput(m_flyWheen.getMotorVoltage().getValueAsDouble());

    this.m_shooterSim.update(SIM_PERIOD_SECONDS);

    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(this.m_shooterSim.getCurrentDrawAmps()));

    double velocityRadPerSec = this.m_shooterSim.getAngularVelocityRadPerSec();

    // Convert Shooter velocity to motor velocity (accounting for gear ratio)
    double motorVelocity = RotationsPerSecond.of((velocityRadPerSec / (2 * Math.PI)) * GEAR_RATIO).in(RotationsPerSecond);

    m_flyWheen.getSimState().setRotorVelocity(motorVelocity);

    // Calculate motor position by integrating velocity over time
    double currentPosition = m_flyWheen.getRotorPosition().getValueAsDouble();
    double newPosition = currentPosition + (motorVelocity * SIM_PERIOD_SECONDS);
    m_flyWheen.getSimState().setRawRotorPosition(newPosition);

    updateVisualization(velocityRadPerSec);

    SmartDashboard.putNumber("Shooter Sim Current (A)", this.m_shooterSim.getCurrentDrawAmps());
  }


  private void updateVisualization(double velocityRadPerSec) {
    ShooterMechanism.update(velocityRadPerSec, SIM_PERIOD_SECONDS, isAtTarget());
  }
}