package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;

public class FeederIOSIM implements FeederIO {

  public FeederIOSIM() {}

  /** {@inheritDoc} */
  @Override
  public void setVoltage(Voltage voltage) {
    Robot.telemetry().log("SimMechanisms/FeederSpeed", voltage.in(Volts) / 12.0);
  }
}
