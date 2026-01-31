package frc.robot.subsystems.feeder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;

@Logged
public interface FeederIO {
  public default void update() {}

  public default void setVelocity(AngularVelocity pVelocity) {}
}
