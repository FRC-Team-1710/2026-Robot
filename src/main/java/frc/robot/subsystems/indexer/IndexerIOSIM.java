package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;

public class IndexerIOSIM implements IndexerIO {

  public IndexerIOSIM() {}

  /** {@inheritDoc} */
  @Override
  public void setVoltage(Voltage voltage) {
    Robot.telemetry().log("SimMechanisms/IndexerSpeed", voltage.in(Volts) / 12.0);
  }
}
