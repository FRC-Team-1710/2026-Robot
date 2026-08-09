package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.utils.FuelSim;

public interface ShooterIO {

  /**
   * Sets the flywheel velocity target for the shooter.
   *
   * @param velocity target flywheel angular velocity
   */
  public default void setTargetVelocity(AngularVelocity velocity) {}

  /**
   * Sets the hood target angle.
   *
   * @param angle target hood angle
   */
  public default void setHoodTarget(Angle angle) {}

  /**
   * Returns the current hood angle measurement.
   *
   * @return current hood position angle
   */
  public default Angle getHoodPosition() {
    return Degrees.of(0);
  }

  /**
   * Sets the stator and supply current limits on flywheel motors (only if needed (hopefully not))
   */
  public default void dynamicCurrentLimit(Current supply, Current stator) {}

  /**
   * Injects the fuel simulator reference for simulation-backed IO implementations.
   *
   * @param fuelSim shooter fuel simulator
   */
  public default void setFuelSim(FuelSim fuelSim) {}
}
