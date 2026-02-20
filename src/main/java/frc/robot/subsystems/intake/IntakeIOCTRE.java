// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.constants.CanIdConstants;
import frc.robot.utils.TalonFXUtil;

@Logged
public class IntakeIOCTRE implements IntakeIO {
  /** Creates a new Intake. */
  @Logged(importance = Importance.CRITICAL)
  private final TalonFX m_intakeMotor;

  @Logged(importance = Importance.CRITICAL)
  private final TalonFX m_deploymentMotor;

  @NotLogged private final DynamicMotionMagicVoltage m_request;

  @Logged(importance = Importance.INFO)
  private Angle m_angleSetpoint;

  @NotLogged private final BaseStatusSignal[] m_intakeSignals;

  @NotLogged private final BaseStatusSignal[] m_deploymentSignals;

  public IntakeIOCTRE() {
    m_intakeMotor = new TalonFX(CanIdConstants.Intake.INTAKE_MOTOR);
    m_deploymentMotor = new TalonFX(CanIdConstants.Intake.DEPLOYMENT_MOTOR);

    m_request = new DynamicMotionMagicVoltage(0, 0, 0).withSlot(0).withEnableFOC(true);

    TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
    m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_motorConfig.Feedback.SensorToMechanismRatio = 1 / 1; // Use the integrated sensor

    m_motorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0625;
    m_motorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.0625;

    // set slot 0 gains
    Slot0Configs m_slot0Configs = m_motorConfig.Slot0;
    m_slot0Configs.kG = 0; // Add 0.0 V output to overcome gravity
    m_slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
    m_slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
    m_slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    m_slot0Configs.kP = 0; // An error of 1 rps results in 0.11 V output
    m_slot0Configs.kI = 0; // no output for integrated error
    m_slot0Configs.kD = 0; // no output for error derivative
    m_slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    m_slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    m_deploymentMotor.getConfigurator().apply(m_motorConfig);
    m_intakeMotor.getConfigurator().apply(m_motorConfig);

    m_deploymentMotor.setPosition(0);

    m_deploymentMotor.getClosedLoopReference().getValue();

    m_intakeSignals = TalonFXUtil.getBasicStatusSignals(m_intakeMotor);
    m_deploymentSignals = TalonFXUtil.getBasicStatusSignals(m_deploymentMotor);

    BaseStatusSignal.setUpdateFrequencyForAll(50, m_intakeSignals);
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_deploymentSignals);

    m_intakeMotor.optimizeBusUtilization();
    m_deploymentMotor.optimizeBusUtilization();
  }

  public void update() {
    BaseStatusSignal.refreshAll(m_intakeSignals);
    BaseStatusSignal.refreshAll(m_deploymentSignals);
  }

  public void setAngle(Angle angle) {
    m_angleSetpoint = angle;
    // m_deploymentMotor.setControl(m_request.withPosition(angle));
    m_deploymentMotor.stopMotor();
  }

  public void setIntakeMotor(double speed) {
    m_intakeMotor.set(speed);
  }

  @NotLogged
  public AngularVelocity getRollerVelocity() {
    return m_intakeMotor.getRotorVelocity().getValue();
  }

  @NotLogged
  public Current getRollerCurrent() {
    return m_intakeMotor.getStatorCurrent().getValue();
  }

  @NotLogged
  public Current getDeploymentCurrent() {
    return m_deploymentMotor.getStatorCurrent().getValue();
  }
}
