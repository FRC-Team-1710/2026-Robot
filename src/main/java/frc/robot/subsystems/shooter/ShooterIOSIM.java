package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.constants.Alliance;
import frc.robot.constants.SubsystemConstants.ShooterConstants;
import frc.robot.utils.FuelSim;
import frc.robot.utils.shooterMath.ShooterMath;

public class ShooterIOSIM implements ShooterIO {

  private AngularVelocity m_velocity = RotationsPerSecond.of(0);
  private Angle m_hoodAngle = Degrees.of(0);

  private FuelSim m_fuelSim;

  // 15 fuel/sec
  private final Debouncer m_shooterDebouncer1 = new Debouncer((1 / 27.5) * 5);
  private final Debouncer m_shooterDebouncer2 = new Debouncer((1 / 27.5) * 5);
  private final Debouncer m_shooterDebouncer3 = new Debouncer((1 / 27.5) * 5);
  private final Debouncer m_shooterDebouncer4 = new Debouncer((1 / 27.5) * 5);
  private final Debouncer m_shooterDebouncer5 = new Debouncer((1 / 27.5) * 5);

  public ShooterIOSIM() {
    SmartDashboard.putNumber("tuning/ShooterMult", 0.13);
  }

  private void update() {
    if (m_fuelSim == null) return;

    if (m_shooterDebouncer1.calculate(m_fuelSim.getCurrentFuelStorage() > 0)
        && m_fuelSim.shouldShoot.getAsBoolean()) {
      if (m_fuelSim.shouldScore.getAsBoolean()) {
        m_fuelSim.spawnFuel(
            new Pose3d(
                    new Pose2d(
                        ShooterMath.currentPose.getTranslation(),
                        ShooterMath.currentSolution.robotHeading().plus(Rotation2d.k180deg)))
                .plus(
                    new Transform3d(
                        -ShooterConstants.Flywheel.Hardware.kShooterOffset.getX(),
                        (Math.random() * (Units.inchesToMeters(12.295) * 2))
                            - Units.inchesToMeters(12.295),
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getZ(),
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getRotation()))
                .getTranslation(),
            new Translation3d(
                ShooterMath.currentSolution.flywheelOmega().in(RotationsPerSecond)
                    * SmartDashboard.getNumber("tuning/ShooterMult", 0),
                new Rotation3d(
                    0,
                    -Degrees.of(90).minus(ShooterMath.currentSolution.hoodAngle()).in(Radians),
                    ShooterMath.currentSolution
                        .robotHeading()
                        .plus(Alliance.redAlliance ? Rotation2d.kZero : Rotation2d.k180deg)
                        .getRadians())));
      } else {
        m_fuelSim.spawnFuel(
            new Pose3d(ShooterMath.currentPose)
                .plus(
                    new Transform3d(
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getX(),
                        (Math.random() * (Units.inchesToMeters(12.295) * 2))
                            - Units.inchesToMeters(12.295),
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getZ(),
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getRotation()))
                .getTranslation(),
            new Translation3d(
                ShooterMath.currentPassingSolution.flywheelOmega().in(RotationsPerSecond)
                    * SmartDashboard.getNumber("tuning/ShooterMult", 0),
                new Rotation3d(
                    0,
                    -Degrees.of(90)
                        .minus(ShooterMath.currentPassingSolution.hoodAngle())
                        .in(Radians),
                    ShooterMath.currentPose.getRotation().getRadians() + Math.PI)));
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
                        ShooterMath.currentPose.getTranslation(),
                        ShooterMath.currentSolution.robotHeading().plus(Rotation2d.k180deg)))
                .plus(
                    new Transform3d(
                        -ShooterConstants.Flywheel.Hardware.kShooterOffset.getX(),
                        (Math.random() * (Units.inchesToMeters(12.295) * 2))
                            - Units.inchesToMeters(12.295),
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getZ(),
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getRotation()))
                .getTranslation(),
            new Translation3d(
                ShooterMath.currentSolution.flywheelOmega().in(RotationsPerSecond)
                    * SmartDashboard.getNumber("tuning/ShooterMult", 0),
                new Rotation3d(
                    0,
                    -Degrees.of(90).minus(ShooterMath.currentSolution.hoodAngle()).in(Radians),
                    ShooterMath.currentSolution
                        .robotHeading()
                        .plus(Alliance.redAlliance ? Rotation2d.kZero : Rotation2d.k180deg)
                        .getRadians())));
      } else {
        m_fuelSim.spawnFuel(
            new Pose3d(ShooterMath.currentPose)
                .plus(
                    new Transform3d(
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getX(),
                        (Math.random() * (Units.inchesToMeters(12.295) * 2))
                            - Units.inchesToMeters(12.295),
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getZ(),
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getRotation()))
                .getTranslation(),
            new Translation3d(
                ShooterMath.currentPassingSolution.flywheelOmega().in(RotationsPerSecond)
                    * SmartDashboard.getNumber("tuning/ShooterMult", 0),
                new Rotation3d(
                    0,
                    -Degrees.of(90)
                        .minus(ShooterMath.currentPassingSolution.hoodAngle())
                        .in(Radians),
                    ShooterMath.currentPose.getRotation().getRadians() + Math.PI)));
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
                        ShooterMath.currentPose.getTranslation(),
                        ShooterMath.currentSolution.robotHeading().plus(Rotation2d.k180deg)))
                .plus(
                    new Transform3d(
                        -ShooterConstants.Flywheel.Hardware.kShooterOffset.getX(),
                        (Math.random() * (Units.inchesToMeters(12.295) * 2))
                            - Units.inchesToMeters(12.295),
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getZ(),
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getRotation()))
                .getTranslation(),
            new Translation3d(
                ShooterMath.currentSolution.flywheelOmega().in(RotationsPerSecond)
                    * SmartDashboard.getNumber("tuning/ShooterMult", 0),
                new Rotation3d(
                    0,
                    -Degrees.of(90).minus(ShooterMath.currentSolution.hoodAngle()).in(Radians),
                    ShooterMath.currentSolution
                        .robotHeading()
                        .plus(Alliance.redAlliance ? Rotation2d.kZero : Rotation2d.k180deg)
                        .getRadians())));
      } else {
        m_fuelSim.spawnFuel(
            new Pose3d(ShooterMath.currentPose)
                .plus(
                    new Transform3d(
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getX(),
                        (Math.random() * (Units.inchesToMeters(12.295) * 2))
                            - Units.inchesToMeters(12.295),
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getZ(),
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getRotation()))
                .getTranslation(),
            new Translation3d(
                ShooterMath.currentPassingSolution.flywheelOmega().in(RotationsPerSecond)
                    * SmartDashboard.getNumber("tuning/ShooterMult", 0),
                new Rotation3d(
                    0,
                    -Degrees.of(90)
                        .minus(ShooterMath.currentPassingSolution.hoodAngle())
                        .in(Radians),
                    ShooterMath.currentPose.getRotation().getRadians() + Math.PI)));
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
                        ShooterMath.currentPose.getTranslation(),
                        ShooterMath.currentSolution.robotHeading().plus(Rotation2d.k180deg)))
                .plus(
                    new Transform3d(
                        -ShooterConstants.Flywheel.Hardware.kShooterOffset.getX(),
                        (Math.random() * (Units.inchesToMeters(12.295) * 2))
                            - Units.inchesToMeters(12.295),
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getZ(),
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getRotation()))
                .getTranslation(),
            new Translation3d(
                ShooterMath.currentSolution.flywheelOmega().in(RotationsPerSecond)
                    * SmartDashboard.getNumber("tuning/ShooterMult", 0),
                new Rotation3d(
                    0,
                    -Degrees.of(90).minus(ShooterMath.currentSolution.hoodAngle()).in(Radians),
                    ShooterMath.currentSolution
                        .robotHeading()
                        .plus(Alliance.redAlliance ? Rotation2d.kZero : Rotation2d.k180deg)
                        .getRadians())));
      } else {
        m_fuelSim.spawnFuel(
            new Pose3d(ShooterMath.currentPose)
                .plus(
                    new Transform3d(
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getX(),
                        (Math.random() * (Units.inchesToMeters(12.295) * 2))
                            - Units.inchesToMeters(12.295),
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getZ(),
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getRotation()))
                .getTranslation(),
            new Translation3d(
                ShooterMath.currentPassingSolution.flywheelOmega().in(RotationsPerSecond)
                    * SmartDashboard.getNumber("tuning/ShooterMult", 0),
                new Rotation3d(
                    0,
                    -Degrees.of(90)
                        .minus(ShooterMath.currentPassingSolution.hoodAngle())
                        .in(Radians),
                    ShooterMath.currentPose.getRotation().getRadians() + Math.PI)));
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
                        ShooterMath.currentPose.getTranslation(),
                        ShooterMath.currentSolution.robotHeading().plus(Rotation2d.k180deg)))
                .plus(
                    new Transform3d(
                        -ShooterConstants.Flywheel.Hardware.kShooterOffset.getX(),
                        (Math.random() * (Units.inchesToMeters(12.295) * 2))
                            - Units.inchesToMeters(12.295),
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getZ(),
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getRotation()))
                .getTranslation(),
            new Translation3d(
                ShooterMath.currentSolution.flywheelOmega().in(RotationsPerSecond)
                    * SmartDashboard.getNumber("tuning/ShooterMult", 0),
                new Rotation3d(
                    0,
                    -Degrees.of(90).minus(ShooterMath.currentSolution.hoodAngle()).in(Radians),
                    ShooterMath.currentSolution
                        .robotHeading()
                        .plus(Alliance.redAlliance ? Rotation2d.kZero : Rotation2d.k180deg)
                        .getRadians())));
      } else {
        m_fuelSim.spawnFuel(
            new Pose3d(ShooterMath.currentPose)
                .plus(
                    new Transform3d(
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getX(),
                        (Math.random() * (Units.inchesToMeters(12.295) * 2))
                            - Units.inchesToMeters(12.295),
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getZ(),
                        ShooterConstants.Flywheel.Hardware.kShooterOffset.getRotation()))
                .getTranslation(),
            new Translation3d(
                ShooterMath.currentPassingSolution.flywheelOmega().in(RotationsPerSecond)
                    * SmartDashboard.getNumber("tuning/ShooterMult", 0),
                new Rotation3d(
                    0,
                    -Degrees.of(90)
                        .minus(ShooterMath.currentPassingSolution.hoodAngle())
                        .in(Radians),
                    ShooterMath.currentPose.getRotation().getRadians() + Math.PI)));
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

    Robot.telemetry().log("FuelSim/FuelLeft", m_fuelSim.getCurrentFuelStorage());
    Robot.telemetry()
        .log(
            "FuelSim/FuelScored", FuelSim.Hub.BLUE_HUB.getScore() + FuelSim.Hub.RED_HUB.getScore());

    Robot.telemetry().log("SimMechanisms/FlywheelSpeed", m_velocity);
    Robot.telemetry().log("SimMechanisms/HoodAngle", m_hoodAngle);
  }

  /** {@inheritDoc} */
  @Override
  public void setTargetVelocity(AngularVelocity velocity) {
    m_velocity = velocity;

    update();
  }

  /** {@inheritDoc} */
  @Override
  public void setHoodTarget(Angle angle) {
    m_hoodAngle =
        Degrees.of(
            MathUtil.clamp(
                angle.in(Degrees),
                ShooterConstants.Hood.Hardware.kHoodMin.in(Degrees),
                ShooterConstants.Hood.Hardware.kHoodMax.in(Degrees)));
  }

  /** {@inheritDoc} */
  @Override
  public Angle getHoodPosition() {
    return m_hoodAngle;
  }

  /** {@inheritDoc} */
  @Override
  public void dynamicCurrentLimit(Current supply, Current stator) {}

  /** {@inheritDoc} */
  @Override
  public void setFuelSim(FuelSim fuelSim) {
    m_fuelSim = fuelSim;
  }
}
