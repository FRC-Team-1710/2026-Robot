package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.MechanismUtil.FlywheelMechanism;
import frc.robot.utils.MechanismUtil.HoodMechanism;

@Logged
public class ShooterIOSIM implements ShooterIO {

  private AngularVelocity m_velocity;
  private Angle m_hoodAngle;

  private final FlywheelMechanism m_flyWheelMechanism;
  private final FlywheelMechanism m_flyWheelFollowerMechanism;

  private final HoodMechanism m_hoodMechanism;

  public ShooterIOSIM() {
    this.m_flyWheelMechanism = new FlywheelMechanism("Flywheel 1", 0.1, -0.25);
    this.m_flyWheelFollowerMechanism = new FlywheelMechanism("Flywheel 2", 0.1, 0.25);

    this.m_hoodMechanism = new HoodMechanism("Hood", 0);
  }

  public void update() {
    this.m_flyWheelMechanism.update(this.m_velocity.in(RadiansPerSecond), 0.02, false);
    this.m_flyWheelMechanism.update(this.m_velocity.in(RadiansPerSecond), 0.02, false);

    this.m_hoodMechanism.update(this.m_hoodAngle);

    SmartDashboard.putData("Flywheel 1", this.m_flyWheelMechanism.getMechanism());
    SmartDashboard.putData("Flywheel 2", this.m_flyWheelFollowerMechanism.getMechanism());

    SmartDashboard.putData("Hood", this.m_hoodMechanism.getMechanism());
  }

  public void stop() {
    this.m_velocity = DegreesPerSecond.of(0);
  }

  public void setTargetVelocity(AngularVelocity velocity) {
    this.m_velocity = velocity;
  }

  public AngularVelocity getVelocity() {
    return this.m_velocity;
  }

  public AngularVelocity getTargetVelocity() {
    return this.m_velocity;
  }

  public void setHoodAngle(Angle angle) {
    this.m_hoodAngle = angle;
  }

  public Angle getHoodAngle() {
    return this.m_hoodAngle;
  }
}
