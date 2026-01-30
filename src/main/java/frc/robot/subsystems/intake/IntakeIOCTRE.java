// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.CanIdConstants;

@Logged
@SuppressWarnings("unused")
public class IntakeIOCTRE implements IntakeIO {
  /** Creates a new Intake. */
  private final TalonFX intakeMotor;

  private final TalonFX deploymentMotor;

  @NotLogged private final MotionMagicVoltage request;

  private Angle angleSetpoint;

  public IntakeIOCTRE() {
    intakeMotor = new TalonFX(CanIdConstants.Intake.INTAKE_MOTOR);
    deploymentMotor = new TalonFX(CanIdConstants.Intake.DEPLOYMENT_MOTOR);

    request = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.Feedback.SensorToMechanismRatio = 1 / 1; // Use the integrated sensor

    // set slot 0 gains
    Slot0Configs slot0Configs = motorConfig.Slot0;
    slot0Configs.kG = 0; // Add 0.0 V output to overcome gravity
    slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative
    slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    MotionMagicConfigs mmConfig = motorConfig.MotionMagic;
    mmConfig.MotionMagicAcceleration =
        400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    mmConfig.MotionMagicCruiseVelocity =
        400; // Target acceleration of 400 rps (0.25 seconds to max)

    deploymentMotor.getConfigurator().apply(motorConfig);
    intakeMotor.getConfigurator().apply(motorConfig);

    deploymentMotor.setPosition(0);

    deploymentMotor.getClosedLoopReference().getValue();
  }

  public void setAngle(Angle angle) {
    angleSetpoint = angle;
    deploymentMotor.setControl(request.withPosition(angle));
  }

  public void setIntakeMotor(double speed) {
    intakeMotor.set(speed);
  }
}
