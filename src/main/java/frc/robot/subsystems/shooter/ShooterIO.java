package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;

public interface ShooterIO {

  public default void stop() {}
  public default void setVelocity(AngularVelocity velocity) {}
}
