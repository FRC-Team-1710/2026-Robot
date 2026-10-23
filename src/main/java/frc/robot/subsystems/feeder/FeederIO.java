package frc.robot.subsystems.feeder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface FeederIO {

  /** Sets feeder motor output voltage */
  public default void setVoltage(Voltage voltage) {}
}
