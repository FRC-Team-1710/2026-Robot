package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.utils.MechanismUtil.FlywheelMechanism;

@Logged
public class ShooterIOSIM implements ShooterIO {

  private final FlywheelMechanism m_flywheelMechanism;

  public ShooterIOSIM() {
    this.m_flywheelMechanism = new FlywheelMechanism("Flywheel 1", 100);
  }

  public void update() {
    this.m_flywheelMechanism.update(10, 0.1, false);
  }

  public void stop() {}

  public void setVelocity(AngularVelocity velocity) {}

  public AngularVelocity getVelocity() {
    return null;
  }

  public void setHoodAngle(Angle angle) {}

  public Angle getHoodAngle() {
    return null;
  }
}
