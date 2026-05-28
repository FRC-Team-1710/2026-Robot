// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.constants.CanIdConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.utils.TalonFXUtil;

/**
 * CTRE (TalonFX) implementation of {@link IntakeIO}.
 *
 * <p>This class configures TalonFX motor controllers for the intake roller and the deployment
 * motor, exposes status signals, and provides methods used by the {@link Intake} subsystem to
 * command motion and read sensor feedback.
 */
public class IntakeIOCTRE implements IntakeIO {
  private final TalonFX m_intakeMotor; // LEFT MOTOR

  private final TalonFX m_intakeMotorFollower; // RIGHT MOTOR

  private final TalonFX m_deploymentMotor;

  private final DynamicMotionMagicVoltage m_deploymentRequest;

  private final VoltageOut m_intakeRequest;

  private Angle m_angleSetpoint;

  /** Cached status signals for the intake TalonFX used to sample sensor values. */
  private final BaseStatusSignal[] m_intakeSignals;

  /** Cached status signals for the intake follower TalonFX used to sample sensor values. */
  private final BaseStatusSignal[] m_intakeFollowerSignals;

  /** Cached status signals for the deployment TalonFX used to sample sensor values. */
  private final BaseStatusSignal[] m_deploymentSignals;

  private final BaseStatusSignal m_deploymentSetpointVelocitySignal;

  public IntakeIOCTRE() {
    m_intakeMotor = new TalonFX(CanIdConstants.Intake.INTAKE_MOTOR);
    m_intakeMotorFollower = new TalonFX(CanIdConstants.Intake.INTAKE_MOTOR_FOLLOWER);
    m_deploymentMotor = new TalonFX(CanIdConstants.Intake.DEPLOYMENT_MOTOR, TunerConstants.kCANBus);

    m_deploymentRequest = new DynamicMotionMagicVoltage(0, 0, 0).withSlot(0).withEnableFOC(true);
    m_intakeRequest = new VoltageOut(0).withEnableFOC(true);

    TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
    m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_motorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0625;
    m_motorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.0625;

    m_motorConfig.CurrentLimits.StatorCurrentLimit = 80;
    m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    m_motorConfig.CurrentLimits.SupplyCurrentLimit = 50; // 55
    m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_intakeMotor.getConfigurator().apply(m_motorConfig);
    m_intakeMotorFollower.getConfigurator().apply(m_motorConfig);

    Slot0Configs m_slot0Configs = m_motorConfig.Slot0;
    m_slot0Configs.kG = 0.4;
    m_slot0Configs.kS = 0.05;
    m_slot0Configs.kV = 6.25;
    m_slot0Configs.kA = 0;
    m_slot0Configs.kP = 0.75;
    m_slot0Configs.kI = 0;
    m_slot0Configs.kD = 0;
    m_slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    m_slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    m_motorConfig.Feedback.SensorToMechanismRatio = 50;
    m_motorConfig.Slot0 = m_slot0Configs;
    m_motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_motorConfig.CurrentLimits.StatorCurrentLimit = 45;
    m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    m_motorConfig.CurrentLimits.SupplyCurrentLimit = 70;
    m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_deploymentMotor.getConfigurator().apply(m_motorConfig);

    m_deploymentMotor.setPosition(0.29);

    m_intakeMotorFollower.setControl(
        new Follower(CanIdConstants.Intake.INTAKE_MOTOR, MotorAlignmentValue.Opposed));

    m_intakeSignals = TalonFXUtil.getBasicStatusSignals(m_intakeMotor);
    m_intakeFollowerSignals = TalonFXUtil.getBasicStatusSignals(m_intakeMotorFollower);
    m_deploymentSignals = TalonFXUtil.getBasicStatusSignals(m_deploymentMotor);

    BaseStatusSignal.setUpdateFrequencyForAll(50, m_intakeSignals);
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_intakeFollowerSignals);
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_deploymentSignals);

    m_deploymentSetpointVelocitySignal = m_deploymentMotor.getClosedLoopReferenceSlope();
    m_deploymentSetpointVelocitySignal.setUpdateFrequency(50);

    m_intakeMotor.optimizeBusUtilization();
    m_intakeMotorFollower.optimizeBusUtilization();
    m_deploymentMotor.optimizeBusUtilization();
  }

  /**
   * Refresh status signals from underlying TalonFX devices. Should be called periodically by the
   * owning subsystem.
   */
  public void update() {
    BaseStatusSignal.refreshAll(m_intakeSignals);
    BaseStatusSignal.refreshAll(m_intakeFollowerSignals);
    BaseStatusSignal.refreshAll(m_deploymentSetpointVelocitySignal);
    BaseStatusSignal.refreshAll(m_deploymentSignals);
  }

  /**
   * Command the deployment motor toward the requested angle.
   *
   * @param angle desired deploy/setpoint angle
   * @param velocity desired deploy/setpoint
   * @param acceleration desired deploy/setpoint
   */
  public void setAngle(Angle angle, double velocity, double acceleration) {
    m_angleSetpoint = angle;
    if (m_deploymentMotor.getPosition(false).getValue().in(Rotations) < 0.0875
        && angle.isEquivalent(IntakeStates.Down.setpoint)) {
      m_deploymentMotor.stopMotor();
    } else {
      m_deploymentMotor.setControl(
          m_deploymentRequest
              .withPosition(angle)
              .withAcceleration(acceleration)
              .withVelocity(velocity));
    }
  }

  /** Returns the closed loop reference slope == 0 */
  public boolean getSetpointReferenceVelocityIsZero() {
    return m_deploymentSetpointVelocitySignal.getValueAsDouble() == 0;
  }

  /**
   * Set the intake roller motor output.
   *
   * @param speed motor output in the range [-1.0, 1.0]
   */
  public void setIntakeMotor(double speed) {
    m_intakeMotor.setControl(m_intakeRequest.withOutput(speed * 12));
  }

  /**
   * @return the rotor/angular velocity of the intake roller motor.
   */
  public AngularVelocity getRollerVelocity() {
    return m_intakeMotor.getRotorVelocity(false).getValue();
  }

  /**
   * @return the angular velocity of the intake follower motor.
   */
  public AngularVelocity getFollowerVelocity() {
    return m_intakeMotorFollower.getVelocity(false).getValue();
  }

  /**
   * @return the stator current draw of the intake roller motor.
   */
  public Current getRollerCurrent() {
    return m_intakeMotor.getStatorCurrent(false).getValue();
  }

  /**
   * @return the stator current draw of the intake follower motor.
   */
  public Current getFollowerCurrent() {
    return m_intakeMotorFollower.getStatorCurrent(false).getValue();
  }

  /**
   * @return the stator current draw of the deployment motor.
   */
  public Current getDeploymentCurrent() {
    return m_deploymentMotor.getStatorCurrent(false).getValue();
  }
}
