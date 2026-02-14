package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.FuelSim;
import frc.robot.utils.MechanismUtil.HoodMechanism;
import frc.robot.utils.MechanismUtil.WheelMechanism;
import frc.robot.utils.shooterMath.ShooterMath;

@Logged
public class ShooterIOSIM implements ShooterIO {

  private AngularVelocity m_velocity;
  private Angle m_hoodAngle;

  private final WheelMechanism m_flyWheelMechanism;
  private final WheelMechanism m_flyWheelFollowerMechanism;

  private final HoodMechanism m_hoodMechanism;

  private FuelSim fuelSim;
  private final Debouncer m_shooterDebouncer = new Debouncer(1 / 12.0); // 12 fuel/sec
  private boolean leftShooter = false;

  public ShooterIOSIM() {
    this.m_flyWheelMechanism = new WheelMechanism("Flywheel 1", 0.1, -0.25, 0.5);
    this.m_flyWheelFollowerMechanism = new WheelMechanism("Flywheel 2", 0.1, 0.25, 0.5);

    this.m_hoodMechanism = new HoodMechanism("Hood", 0);
  }

  public void update() {
    this.m_flyWheelMechanism.update(this.m_velocity.in(RadiansPerSecond), 0.02, false);
    this.m_flyWheelMechanism.update(this.m_velocity.in(RadiansPerSecond), 0.02, false);

    this.m_hoodMechanism.update(this.m_hoodAngle);

    SmartDashboard.putData("Flywheel 1", this.m_flyWheelMechanism.getMechanism());
    SmartDashboard.putData("Flywheel 2", this.m_flyWheelFollowerMechanism.getMechanism());

    SmartDashboard.putData("Hood", this.m_hoodMechanism.getMechanism());

    if (fuelSim == null) return;

    if (m_shooterDebouncer.calculate(fuelSim.getCurrentFuelStorage() > 0)
        && getVelocity().isNear(ShooterMath.getShooterRPM(), 0.1)) {
      double mult = 1 / (Math.pow(ShooterMath.calculateDistance(), 0.15));
      if (leftShooter) {
        fuelSim.spawnFuel(
            ShooterMath.getRobotPose().plus(ShooterConstants.kLEFT_SHOOTER_OFFSET).getTranslation(),
            ShooterMath.findShooterVelocity3d().times(mult).inverse().toTranslation3d());
      } else {
        fuelSim.spawnFuel(
            ShooterMath.getRobotPose()
                .plus(ShooterConstants.kRIGHT_SHOOTER_OFFSET)
                .getTranslation(),
            ShooterMath.findShooterVelocity3d().times(mult).inverse().toTranslation3d());
      }
      fuelSim.removeFuelFromStorage(1);
      leftShooter = !leftShooter;
      m_shooterDebouncer.calculate(false);
    }
  }

  public void stop() {
    this.m_velocity = DegreesPerSecond.of(0);
  }

  public void setTargetVelocity(AngularVelocity pVelocity) {
    this.m_velocity = pVelocity;
  }

  public AngularVelocity getVelocity() {
    return this.m_velocity;
  }

  public AngularVelocity getTargetVelocity() {
    return this.m_velocity;
  }

  public void setHoodAngle(Angle pAngle) {
    this.m_hoodAngle = pAngle;
  }

  public Angle getHoodAngle() {
    return this.m_hoodAngle;
  }

  public void setFuelSim(FuelSim fuelSim) {
    this.fuelSim = fuelSim;
  }
}
