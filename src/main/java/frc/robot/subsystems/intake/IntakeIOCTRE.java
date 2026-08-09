package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.SubsystemConstants.IntakeConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;

public class IntakeIOCTRE implements IntakeIO {

  private final TalonFX m_rollerLeft;
  private final TalonFX m_rollerRight;
  private final TalonFX m_deploymentMotor;

  private final DynamicMotionMagicVoltage m_mmVoltage =
      new DynamicMotionMagicVoltage(0, 0, 0).withEnableFOC(true);

  private final VoltageOut m_voltage = new VoltageOut(0).withEnableFOC(true);

  private final VoltageOut m_voltageOut = new VoltageOut(0).withEnableFOC(true);

  public IntakeIOCTRE() {
    m_rollerLeft = new TalonFX(CanIdConstants.Intake.ROLLER_LEFT);
    m_rollerRight = new TalonFX(CanIdConstants.Intake.ROLLER_RIGHT);
    m_deploymentMotor = new TalonFX(CanIdConstants.Intake.DEPLOYMENT_MOTOR, TunerConstants.kCANBus);

    TalonFXUtil.applyConfig(
        IntakeConstants.Rollers.Software.Config.kConfig, m_rollerLeft, m_rollerRight);
    TalonFXUtil.applyConfig(IntakeConstants.Deployment.Software.Config.kConfig, m_deploymentMotor);

    m_deploymentMotor.setPosition(IntakeConstants.Deployment.Hardware.kStowAngle);

    TalonFXUtil.optimizeForBasicStatusSignals(m_rollerLeft, m_rollerRight);
    TalonFXUtil.optimizeForPIDStatusSignals(m_deploymentMotor);
  }

  /** {@inheritDoc} */
  @Override
  public void setAngle(Angle angle, double acceleration, double velocity) {
    if (m_deploymentMotor.getPosition().getValue().in(Rotations)
            < IntakeConstants.Deployment.Software.kLowerThreshold.in(Rotations)
        && angle.isEquivalent(IntakeConstants.Deployment.Hardware.kDeployAngle)) {
      m_deploymentMotor.setControl(
          m_voltage.withOutput(Volts.of(DriverStation.isAutonomous() ? 0.0 : -0.25)));
    } else {
      m_deploymentMotor.setControl(
          m_mmVoltage.withPosition(angle).withAcceleration(acceleration).withVelocity(velocity));
    }
  }

  /** {@inheritDoc} */
  @Override
  public void setIntakeVoltage(Voltage voltage) {
    // if (m_deploymentMotor
    //     .getPosition()
    //     .getValue()
    //     .gte(
    //         IntakeConstants.Deployment.Hardware.kDeployAngle
    //             .plus(IntakeConstants.Deployment.Hardware.kStowAngle)
    //             .div(2.0))) {
    m_rollerLeft.setControl(m_voltageOut.withOutput(voltage));
    m_rollerRight.setControl(m_voltageOut.withOutput(voltage.times(-1)));
    // } else {
    //   m_rollerLeft.stopMotor();
    //   m_rollerRight.stopMotor();
    // }
  }
}
