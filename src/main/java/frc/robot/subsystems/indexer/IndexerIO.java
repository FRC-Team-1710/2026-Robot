package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.Voltage;

public interface IndexerIO {

  /** Sets the voltage for the indexer motor */
  public default void setVoltage(Voltage voltage) {}
}
