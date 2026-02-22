package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.FuelSim;
import frc.robot.utils.MechanismUtil.HoodMechanism;
import frc.robot.utils.MechanismUtil.WheelMechanism;
import frc.robot.utils.shooterMath.ShooterMath2;

@Logged
public class ShooterIOSIM implements ShooterIO {

  private AngularVelocity m_velocity = RotationsPerSecond.of(0);
  private Angle m_hoodAngle = Degrees.of(0);

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

    SmartDashboard.putNumber("ShooterMult", 0.205);
  }

  public void update(double dtSeconds) {
    this.m_flyWheelMechanism.update(this.m_velocity.in(RadiansPerSecond), dtSeconds, false);
    this.m_flyWheelFollowerMechanism.update(this.m_velocity.in(RadiansPerSecond), dtSeconds, false);

    this.m_hoodMechanism.update(this.m_hoodAngle);

    SmartDashboard.putData("Flywheel 1", this.m_flyWheelMechanism.getMechanism());
    SmartDashboard.putData("Flywheel 2", this.m_flyWheelFollowerMechanism.getMechanism());

    SmartDashboard.putData("Hood", this.m_hoodMechanism.getMechanism());

    if (fuelSim == null) return;

    if (m_shooterDebouncer.calculate(fuelSim.getCurrentFuelStorage() > 0)
        && getVelocity().isNear(ShooterMath2.currentSolution.shooterLeft().flywheelOmega(), 0.1)) {
      if (leftShooter) {
        fuelSim.spawnFuel(
            new Pose3d(
                    new Pose2d(
                        ShooterMath2.currentPose.getTranslation(),
                        ShooterMath2.currentSolution.robotHeading()))
                .plus(ShooterConstants.kLEFT_SHOOTER_OFFSET)
                .getTranslation(),
            new Translation3d(
                    ShooterMath2.currentSolution
                            .shooterLeft()
                            .flywheelOmega()
                            .in(RotationsPerSecond)
                        * SmartDashboard.getNumber("ShooterMult", 0),
                    new Rotation3d(
                        0,
                        -Degrees.of(90)
                            .minus(ShooterMath2.currentSolution.shooterLeft().hoodAngle())
                            .in(Radians),
                        ShooterMath2.currentSolution.robotHeading().getRadians()))
                .plus(
                    new Translation3d(
                        ShooterMath2.currentSpeeds.vxMetersPerSecond,
                        ShooterMath2.currentSpeeds.vyMetersPerSecond,
                        0)));
      } else {
        fuelSim.spawnFuel(
            new Pose3d(
                    new Pose2d(
                        ShooterMath2.currentPose.getTranslation(),
                        ShooterMath2.currentSolution.robotHeading()))
                .plus(ShooterConstants.kRIGHT_SHOOTER_OFFSET)
                .getTranslation(),
            new Translation3d(
                    ShooterMath2.currentSolution
                            .shooterRight()
                            .flywheelOmega()
                            .in(RotationsPerSecond)
                        * SmartDashboard.getNumber("ShooterMult", 0),
                    new Rotation3d(
                        0,
                        -Degrees.of(90)
                            .minus(ShooterMath2.currentSolution.shooterRight().hoodAngle())
                            .in(Radians),
                        ShooterMath2.currentSolution.robotHeading().getRadians()))
                .plus(
                    new Translation3d(
                        ShooterMath2.currentSpeeds.vxMetersPerSecond,
                        ShooterMath2.currentSpeeds.vyMetersPerSecond,
                        0)));
      }
      // fuelSim.removeFuelFromStorage(1);
      leftShooter = !leftShooter;
      m_shooterDebouncer.calculate(false);
    }
  }

  public void stop() {
    this.m_velocity = DegreesPerSecond.of(0);
  }

  public void setLeftTargetVelocity(AngularVelocity pVelocity) {
    this.m_velocity = pVelocity;
  }

  public AngularVelocity getVelocity() {
    return this.m_velocity;
  }

  public AngularVelocity getTargetVelocity() {
    return this.m_velocity;
  }

  public void setLeftHoodTarget(Angle pAngle) {
    this.m_hoodAngle =
        Degrees.of(
            MathUtil.clamp(
                pAngle.magnitude(), ShooterConstants.HOOD_MIN, ShooterConstants.HOOD_MAX));
  }

  public Angle getLeftHoodPosition() {
    return this.m_hoodAngle;
  }

  public void setFuelSim(FuelSim fuelSim) {
    this.fuelSim = fuelSim;
  }
}
