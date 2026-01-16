// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class IntakeIOCTRE implements IntakeIO {
  /** Creates a new Intake. */
  @Logged(name = "OOO", importance = Importance.CRITICAL)
  private double deploymentMotorGearRatio = 1/1;
  
  public final TalonFX intakeMotor;
  public final TalonFX deploymentMotor;
  
  private MotionMagicVoltage MMRequest;
  
  private Angle angleSetpoint;

  TalonFXConfiguration motorConfig;
  public IntakeIOCTRE() {
    intakeMotor = new TalonFX(1);
    deploymentMotor = new TalonFX(2);
    
    MMRequest = new MotionMagicVoltage(0).withSlot(0);

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
    deploymentMotor.getConfigurator().apply(motorConfig);
    intakeMotor.getConfigurator().apply(motorConfig);

    deploymentMotor.setPosition(0);
  }

  public void setAngle(Angle angle) {
    angleSetpoint = angle;
    deploymentMotor.setControl(MMRequest.withPosition(angle.times(deploymentMotorGearRatio)));
  }

  public void setIntakeMotor(double speed) {
    intakeMotor.set(speed);
  }
}
