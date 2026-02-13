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
  private final WheelMechanism m_feederMotorFollowerMechanism;

  public FeederIOSIM() {
    this.m_feederMotorMechanism = new WheelMechanism("Feeder L", 0.05, 0.15, 0.35);
    this.m_feederMotorFollowerMechanism = new WheelMechanism("Feeder R", 0.05, -0.15, 0.35);
  }

  public void update() {
    this.m_feederMotorMechanism.update(
        RPM.of(this.m_velocity * 6000).in(RadiansPerSecond), 0.02, false);
    this.m_feederMotorFollowerMechanism.update(
        RPM.of(this.m_velocity * 6000).in(RadiansPerSecond), 0.02, false);

    SmartDashboard.putData("Feeder 1", this.m_feederMotorMechanism.getMechanism());
    SmartDashboard.putData("Feeder 2", this.m_feederMotorFollowerMechanism.getMechanism());
  }

  public void setFeeder(double percent) {
    this.m_velocity = percent;
  }
}
