package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public interface IntakeIO {

  /**
   * Command the deployment/arm to the requested angle setpoint.
   *
   * @param angle desired arm angle
   * @param velocity desired velocity
   * @param acceleration desired acceleration
   */
  public default void setAngle(Angle angle, double acceleration, double velocity) {}

  /** Sets the intake motors' voltage */
  public default void setIntakeVoltage(Voltage voltage) {}
}
