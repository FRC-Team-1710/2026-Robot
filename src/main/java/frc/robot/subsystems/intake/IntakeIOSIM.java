package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;

public class IntakeIOSIM implements IntakeIO {

  public IntakeIOSIM() {}

  /** {@inheritDoc} */
  @Override
  public void setAngle(Angle angle, double acceleration, double velocity) {
    Robot.telemetry().log("SimMechanisms/IntakeAngle", angle);
  }

  /** {@inheritDoc} */
  @Override
  public void setIntakeVoltage(Voltage voltage) {
    Robot.telemetry().log("SimMechanisms/IntakeVoltage", voltage);
  }
}
