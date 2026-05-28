package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.FuelSim;
import frc.robot.utils.MechanismUtil.HoodMechanism;
import frc.robot.utils.MechanismUtil.WheelMechanism;
import frc.robot.utils.shooterMath.ShooterMath4;
import org.littletonrobotics.junction.Logger;

/** Simulation implementation of shooter IO. */
public class ShooterIOSIM implements ShooterIO {

  private AngularVelocity m_velocity = RotationsPerSecond.of(0);
  private Angle m_hoodAngle = Degrees.of(0);

  private final WheelMechanism m_flyWheelMechanism;
  private final WheelMechanism m_flyWheelFollowerMechanism;

  private final HoodMechanism m_hoodMechanism;

  private FuelSim m_fuelSim;
  private final Debouncer m_shooterDebouncer1 = new Debouncer((1 / 15.0) * 5); // 15 fuel/sec
  private final Debouncer m_shooterDebouncer2 = new Debouncer((1 / 15.0) * 5); // 15 fuel/sec
  private final Debouncer m_shooterDebouncer3 = new Debouncer((1 / 15.0) * 5); // 15 fuel/sec
  private final Debouncer m_shooterDebouncer4 = new Debouncer((1 / 15.0) * 5); // 15 fuel/sec
  private final Debouncer m_shooterDebouncer5 = new Debouncer((1 / 15.0) * 5); // 15 fuel/sec

  /** Constructs the simulated shooter IO and visualization mechanisms. */
  public ShooterIOSIM() {
    this.m_flyWheelMechanism = new WheelMechanism("Flywheel 1", 0.1, -0.25, 0.5);
    this.m_flyWheelFollowerMechanism = new WheelMechanism("Flywheel 2", 0.1, 0.25, 0.5);

    this.m_hoodMechanism = new HoodMechanism("Hood", 0);

    SmartDashboard.putNumber("tuning/ShooterMult", 0.201);
  }

  /** {@inheritDoc} */
  @Override
  public void update(double dtSeconds) {
    this.m_flyWheelMechanism.update(this.m_velocity.in(RadiansPerSecond), dtSeconds, false);
    this.m_flyWheelFollowerMechanism.update(this.m_velocity.in(RadiansPerSecond), dtSeconds, false);

    this.m_hoodMechanism.update(this.m_hoodAngle);

    SmartDashboard.putData("Flywheel 1", this.m_flyWheelMechanism.getMechanism());
    SmartDashboard.putData("Flywheel 2", this.m_flyWheelFollowerMechanism.getMechanism());

    SmartDashboard.putData("Hood", this.m_hoodMechanism.getMechanism());

    if (m_fuelSim == null) return;

    if (m_shooterDebouncer1.calculate(m_fuelSim.getCurrentFuelStorage() > 0)
        && m_fuelSim.shouldShoot.getAsBoolean()) {
      if (m_fuelSim.shouldScore.getAsBoolean()) {
        m_fuelSim.spawnFuel(
            new Pose3d(
                    new Pose2d(
                        ShooterMath4.currentPose.getTranslation(),
                        ShooterMath4.currentSolution.robotHeading()))
                .plus(
                    new Transform3d(
                        ShooterConstants.kSHOOTER_OFFSET.getX(),
                        (Math.random() * (Units.inchesToMeters(12.295) * 2))
                            - Units.inchesToMeters(12.295),
                        ShooterConstants.kSHOOTER_OFFSET.getZ(),
                        ShooterConstants.kSHOOTER_OFFSET.getRotation()))
                .getTranslation(),
            new Translation3d(
                ShooterMath4.currentSolution.flywheelOmega().in(RotationsPerSecond)
                    * SmartDashboard.getNumber("tuning/ShooterMult", 0),
                new Rotation3d(
                    0,
                    -Degrees.of(90).minus(ShooterMath4.currentSolution.hoodAngle()).in(Radians),
                    ShooterMath4.currentSolution.robotHeading().getRadians())));
      } else {
        m_fuelSim.spawnFuel(
            new Pose3d(ShooterMath4.currentPose)
                .plus(
                    new Transform3d(
                        ShooterConstants.kSHOOTER_OFFSET.getX(),
                        (Math.random() * (Units.inchesToMeters(12.295) * 2))
                            - Units.inchesToMeters(12.295),
                        ShooterConstants.kSHOOTER_OFFSET.getZ(),
                        ShooterConstants.kSHOOTER_OFFSET.getRotation()))
                .getTranslation(),
            new Translation3d(
                ShooterMath4.currentSolution.flywheelOmega().in(RotationsPerSecond)
                    * SmartDashboard.getNumber("tuning/ShooterMult", 0),
                new Rotation3d(
                    0,
                    -Degrees.of(90).minus(ShooterMath4.currentSolution.hoodAngle()).in(Radians),
                    ShooterMath4.currentPose.getRotation().getRadians() + Math.PI)));
      }
      m_fuelSim.removeFuelFromStorage(1);
      m_shooterDebouncer1.setDebounceTime(((1.0 / 15.0) * 5) * (Math.random() * 0.5 + 0.75));
      m_shooterDebouncer1.calculate(false);
    }

    if (m_shooterDebouncer2.calculate(m_fuelSim.getCurrentFuelStorage() > 0)
        && m_fuelSim.shouldShoot.getAsBoolean()) {
      if (m_fuelSim.shouldScore.getAsBoolean()) {
        m_fuelSim.spawnFuel(
            new Pose3d(
                    new Pose2d(
                        ShooterMath4.currentPose.getTranslation(),
                        ShooterMath4.currentSolution.robotHeading()))
                .plus(
                    new Transform3d(
                        ShooterConstants.kSHOOTER_OFFSET.getX(),
                        (Math.random() * (Units.inchesToMeters(12.295) * 2))
                            - Units.inchesToMeters(12.295),
                        ShooterConstants.kSHOOTER_OFFSET.getZ(),
                        ShooterConstants.kSHOOTER_OFFSET.getRotation()))
                .getTranslation(),
            new Translation3d(
                ShooterMath4.currentSolution.flywheelOmega().in(RotationsPerSecond)
                    * SmartDashboard.getNumber("tuning/ShooterMult", 0),
                new Rotation3d(
                    0,
                    -Degrees.of(90).minus(ShooterMath4.currentSolution.hoodAngle()).in(Radians),
                    ShooterMath4.currentSolution.robotHeading().getRadians())));
      } else {
        m_fuelSim.spawnFuel(
            new Pose3d(ShooterMath4.currentPose)
                .plus(
                    new Transform3d(
                        ShooterConstants.kSHOOTER_OFFSET.getX(),
                        (Math.random() * (Units.inchesToMeters(12.295) * 2))
                            - Units.inchesToMeters(12.295),
                        ShooterConstants.kSHOOTER_OFFSET.getZ(),
                        ShooterConstants.kSHOOTER_OFFSET.getRotation()))
                .getTranslation(),
            new Translation3d(
                ShooterMath4.currentSolution.flywheelOmega().in(RotationsPerSecond)
                    * SmartDashboard.getNumber("tuning/ShooterMult", 0),
                new Rotation3d(
                    0,
                    -Degrees.of(90).minus(ShooterMath4.currentSolution.hoodAngle()).in(Radians),
                    ShooterMath4.currentPose.getRotation().getRadians() + Math.PI)));
      }
      m_fuelSim.removeFuelFromStorage(1);
      m_shooterDebouncer2.setDebounceTime(((1.0 / 15.0) * 5) * (Math.random() * 0.5 + 0.75));
      m_shooterDebouncer2.calculate(false);
    }

    if (m_shooterDebouncer3.calculate(m_fuelSim.getCurrentFuelStorage() > 0)
        && m_fuelSim.shouldShoot.getAsBoolean()) {
      if (m_fuelSim.shouldScore.getAsBoolean()) {
        m_fuelSim.spawnFuel(
            new Pose3d(
                    new Pose2d(
                        ShooterMath4.currentPose.getTranslation(),
                        ShooterMath4.currentSolution.robotHeading()))
                .plus(
                    new Transform3d(
                        ShooterConstants.kSHOOTER_OFFSET.getX(),
                        (Math.random() * (Units.inchesToMeters(12.295) * 2))
                            - Units.inchesToMeters(12.295),
                        ShooterConstants.kSHOOTER_OFFSET.getZ(),
                        ShooterConstants.kSHOOTER_OFFSET.getRotation()))
                .getTranslation(),
            new Translation3d(
                ShooterMath4.currentSolution.flywheelOmega().in(RotationsPerSecond)
                    * SmartDashboard.getNumber("tuning/ShooterMult", 0),
                new Rotation3d(
                    0,
                    -Degrees.of(90).minus(ShooterMath4.currentSolution.hoodAngle()).in(Radians),
                    ShooterMath4.currentSolution.robotHeading().getRadians())));
      } else {
        m_fuelSim.spawnFuel(
            new Pose3d(ShooterMath4.currentPose)
                .plus(
                    new Transform3d(
                        ShooterConstants.kSHOOTER_OFFSET.getX(),
                        (Math.random() * (Units.inchesToMeters(12.295) * 2))
                            - Units.inchesToMeters(12.295),
                        ShooterConstants.kSHOOTER_OFFSET.getZ(),
                        ShooterConstants.kSHOOTER_OFFSET.getRotation()))
                .getTranslation(),
            new Translation3d(
                ShooterMath4.currentSolution.flywheelOmega().in(RotationsPerSecond)
                    * SmartDashboard.getNumber("tuning/ShooterMult", 0),
                new Rotation3d(
                    0,
                    -Degrees.of(90).minus(ShooterMath4.currentSolution.hoodAngle()).in(Radians),
                    ShooterMath4.currentPose.getRotation().getRadians() + Math.PI)));
      }
      m_fuelSim.removeFuelFromStorage(1);
      m_shooterDebouncer3.setDebounceTime(((1.0 / 15.0) * 5) * (Math.random() * 0.5 + 0.75));
      m_shooterDebouncer3.calculate(false);
    }

    if (m_shooterDebouncer4.calculate(m_fuelSim.getCurrentFuelStorage() > 0)
        && m_fuelSim.shouldShoot.getAsBoolean()) {
      if (m_fuelSim.shouldScore.getAsBoolean()) {
        m_fuelSim.spawnFuel(
            new Pose3d(
                    new Pose2d(
                        ShooterMath4.currentPose.getTranslation(),
                        ShooterMath4.currentSolution.robotHeading()))
                .plus(
                    new Transform3d(
                        ShooterConstants.kSHOOTER_OFFSET.getX(),
                        (Math.random() * (Units.inchesToMeters(12.295) * 2))
                            - Units.inchesToMeters(12.295),
                        ShooterConstants.kSHOOTER_OFFSET.getZ(),
                        ShooterConstants.kSHOOTER_OFFSET.getRotation()))
                .getTranslation(),
            new Translation3d(
                ShooterMath4.currentSolution.flywheelOmega().in(RotationsPerSecond)
                    * SmartDashboard.getNumber("tuning/ShooterMult", 0),
                new Rotation3d(
                    0,
                    -Degrees.of(90).minus(ShooterMath4.currentSolution.hoodAngle()).in(Radians),
                    ShooterMath4.currentSolution.robotHeading().getRadians())));
      } else {
        m_fuelSim.spawnFuel(
            new Pose3d(ShooterMath4.currentPose)
                .plus(
                    new Transform3d(
                        ShooterConstants.kSHOOTER_OFFSET.getX(),
                        (Math.random() * (Units.inchesToMeters(12.295) * 2))
                            - Units.inchesToMeters(12.295),
                        ShooterConstants.kSHOOTER_OFFSET.getZ(),
                        ShooterConstants.kSHOOTER_OFFSET.getRotation()))
                .getTranslation(),
            new Translation3d(
                ShooterMath4.currentSolution.flywheelOmega().in(RotationsPerSecond)
                    * SmartDashboard.getNumber("tuning/ShooterMult", 0),
                new Rotation3d(
                    0,
                    -Degrees.of(90).minus(ShooterMath4.currentSolution.hoodAngle()).in(Radians),
                    ShooterMath4.currentPose.getRotation().getRadians() + Math.PI)));
      }
      m_fuelSim.removeFuelFromStorage(1);
      m_shooterDebouncer4.setDebounceTime(((1.0 / 15.0) * 5) * (Math.random() * 0.5 + 0.75));
      m_shooterDebouncer4.calculate(false);
    }

    if (m_shooterDebouncer5.calculate(m_fuelSim.getCurrentFuelStorage() > 0)
        && m_fuelSim.shouldShoot.getAsBoolean()) {
      if (m_fuelSim.shouldScore.getAsBoolean()) {
        m_fuelSim.spawnFuel(
            new Pose3d(
                    new Pose2d(
                        ShooterMath4.currentPose.getTranslation(),
                        ShooterMath4.currentSolution.robotHeading()))
                .plus(
                    new Transform3d(
                        ShooterConstants.kSHOOTER_OFFSET.getX(),
                        (Math.random() * (Units.inchesToMeters(12.295) * 2))
                            - Units.inchesToMeters(12.295),
                        ShooterConstants.kSHOOTER_OFFSET.getZ(),
                        ShooterConstants.kSHOOTER_OFFSET.getRotation()))
                .getTranslation(),
            new Translation3d(
                ShooterMath4.currentSolution.flywheelOmega().in(RotationsPerSecond)
                    * SmartDashboard.getNumber("tuning/ShooterMult", 0),
                new Rotation3d(
                    0,
                    -Degrees.of(90).minus(ShooterMath4.currentSolution.hoodAngle()).in(Radians),
                    ShooterMath4.currentSolution.robotHeading().getRadians())));
      } else {
        m_fuelSim.spawnFuel(
            new Pose3d(ShooterMath4.currentPose)
                .plus(
                    new Transform3d(
                        ShooterConstants.kSHOOTER_OFFSET.getX(),
                        (Math.random() * (Units.inchesToMeters(12.295) * 2))
                            - Units.inchesToMeters(12.295),
                        ShooterConstants.kSHOOTER_OFFSET.getZ(),
                        ShooterConstants.kSHOOTER_OFFSET.getRotation()))
                .getTranslation(),
            new Translation3d(
                ShooterMath4.currentSolution.flywheelOmega().in(RotationsPerSecond)
                    * SmartDashboard.getNumber("tuning/ShooterMult", 0),
                new Rotation3d(
                    0,
                    -Degrees.of(90).minus(ShooterMath4.currentSolution.hoodAngle()).in(Radians),
                    ShooterMath4.currentPose.getRotation().getRadians() + Math.PI)));
      }
      m_fuelSim.removeFuelFromStorage(1);
      m_shooterDebouncer5.setDebounceTime(((1.0 / 15.0) * 5) * (Math.random() * 0.5 + 0.75));
      m_shooterDebouncer5.calculate(false);
    }

    if (!m_fuelSim.shouldShoot.getAsBoolean()) {
      m_shooterDebouncer1.calculate(false);
      m_shooterDebouncer2.calculate(false);
      m_shooterDebouncer3.calculate(false);
      m_shooterDebouncer4.calculate(false);
      m_shooterDebouncer5.calculate(false);
      m_shooterDebouncer1.setDebounceTime(((1.0 / 15.0) * 5) * (Math.random() * 0.5 + 0.75));
      m_shooterDebouncer2.setDebounceTime(((1.0 / 15.0) * 5) * (Math.random() * 0.5 + 0.75));
      m_shooterDebouncer3.setDebounceTime(((1.0 / 15.0) * 5) * (Math.random() * 0.5 + 0.75));
      m_shooterDebouncer4.setDebounceTime(((1.0 / 15.0) * 5) * (Math.random() * 0.5 + 0.75));
      m_shooterDebouncer5.setDebounceTime(((1.0 / 15.0) * 5) * (Math.random() * 0.5 + 0.75));
    }

    Logger.recordOutput("FuelLeft", m_fuelSim.getCurrentFuelStorage());
    Logger.recordOutput("FuelScored", FuelSim.Hub.BLUE_HUB.getScore());
  }

  /** {@inheritDoc} */
  @Override
  public void stop() {
    this.m_velocity = RotationsPerSecond.of(0);
  }

  /** {@inheritDoc} */
  @Override
  public void setTargetVelocity(AngularVelocity pVelocity) {
    this.m_velocity = pVelocity;
  }

  /** {@inheritDoc} */
  @Override
  public AngularVelocity getVelocity() {
    return this.m_velocity;
  }

  /** {@inheritDoc} */
  @Override
  public void setHoodTarget(Angle pAngle) {
    this.m_hoodAngle =
        Degrees.of(
            MathUtil.clamp(
                pAngle.in(Degrees), ShooterConstants.HOOD_MIN, ShooterConstants.HOOD_MAX));
  }

  /** {@inheritDoc} */
  @Override
  public Angle getHoodPosition() {
    return this.m_hoodAngle;
  }

  /** Sets the fuel simulation instance for this shooter IO. */
  @Override
  public void setFuelSim(FuelSim fuelSim) {
    this.m_fuelSim = fuelSim;
  }
}
