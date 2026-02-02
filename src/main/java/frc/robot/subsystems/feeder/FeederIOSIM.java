package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.MechanismUtil.WheelMechanism;

@Logged
public class FeederIOSIM implements FeederIO {

  private AngularVelocity m_velocity;

  private final WheelMechanism m_feederMotorMechanism;
  private final WheelMechanism m_feederMotorFollowerMechanism;

  public FeederIOSIM() {
    this.m_feederMotorMechanism = new WheelMechanism("Feeder L", 0.05, 0.15, 0.35);
    this.m_feederMotorFollowerMechanism = new WheelMechanism("Feeder R", 0.05, -0.15, 0.35);
  }

  public void update() {
    this.m_feederMotorMechanism.update(this.m_velocity.in(RadiansPerSecond), 0.02, false);
    this.m_feederMotorFollowerMechanism.update(this.m_velocity.in(RadiansPerSecond), 0.02, false);

    SmartDashboard.putData("Feeder 1", this.m_feederMotorMechanism.getMechanism());
    SmartDashboard.putData("Feeder 2", this.m_feederMotorFollowerMechanism.getMechanism());
  }

  public void setVelocity(boolean clockwise) {
    this.m_velocity = clockwise ? DegreesPerSecond.of(0.25) : DegreesPerSecond.of(-0.25);
  }

  public void stop() {
    this.m_velocity = DegreesPerSecond.of(0);
  }
}
