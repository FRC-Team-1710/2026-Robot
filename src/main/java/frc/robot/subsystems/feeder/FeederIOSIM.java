package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.MechanismUtil.WheelMechanism;

@Logged
public class FeederIOSIM implements FeederIO {

  private double m_velocity;

  private final WheelMechanism m_feederMotorMechanism;

  public FeederIOSIM() {
    this.m_feederMotorMechanism = new WheelMechanism("Feeder L", 0.05, 0.15, 0.35);
  }

  /** {@inheritDoc} */
  public void update(double dtSeconds) {
    this.m_feederMotorMechanism.update(
        RPM.of(this.m_velocity * 6000).in(RadiansPerSecond), dtSeconds, false);
    SmartDashboard.putData("Feeder 1", this.m_feederMotorMechanism.getMechanism());
  }

  /** {@inheritDoc} */
  public void setFeeder(double percent) {
    this.m_velocity = percent;
  }
}
