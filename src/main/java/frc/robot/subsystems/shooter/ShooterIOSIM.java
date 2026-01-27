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

  private AngularVelocity velocity;
  private Angle hoodAngle;

  private final FlywheelMechanism flyWheelMechanism;
  private final FlywheelMechanism flyWheelFollowerMechanism;

  private final HoodMechanism hoodMechanism;

  public ShooterIOSIM() {
    this.flyWheelMechanism = new FlywheelMechanism("Flywheel 1", 0.1, -0.25);
    this.flyWheelFollowerMechanism = new FlywheelMechanism("Flywheel 2", 0.1, 0.25);

    this.hoodMechanism = new HoodMechanism("Hood", 0);
  }

  public void update() {
    this.flyWheelMechanism.update(this.velocity.in(RadiansPerSecond), 0.02, false);
    this.flyWheelMechanism.update(this.velocity.in(RadiansPerSecond), 0.02, false);

    this.hoodMechanism.update(this.hoodAngle);

    SmartDashboard.putData("Flywheel 1", this.flyWheelMechanism.getMechanism());
    SmartDashboard.putData("Flywheel 2", this.flyWheelFollowerMechanism.getMechanism());

    SmartDashboard.putData("Hood", this.hoodMechanism.getMechanism());
  }

  public void stop() {
    this.velocity = DegreesPerSecond.of(0);
  }

  public void setTargetVelocity(AngularVelocity velocity) {
    this.velocity = velocity;
  }

  public AngularVelocity getVelocity() {
    return this.velocity;
  }

  public AngularVelocity getTargetVelocity() {
    return this.velocity;
  }

  public void setHoodAngle(Angle angle) {
    this.hoodAngle = angle;
  }

  public Angle getHoodAngle() {
    return this.hoodAngle;
  }
}
