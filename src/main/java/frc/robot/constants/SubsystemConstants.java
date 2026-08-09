package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public class SubsystemConstants {
  public class IntakeConstants { // Intake
    public class Deployment { // Deployment
      public class Hardware {
        public static final Angle kStowAngle = Rotations.of(0.29);
        public static final Angle kDeployAngle = Rotations.of(0.0);
      }

      public class Software {
        public static final double kDeployAcceleration = 1.5;
        public static final double kDeployVelocity = 1.75;
        public static final double kShootingStowVelocity = 1.25;

        public static final Angle kLowerThreshold = Rotations.of(0.0875);

        public class Config {
          public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
          public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;

          public static final double kSensorToMechanismRatio = 50.0;

          public static final Current kStatorCurrentLimit = Amps.of(45.0);
          public static final Current kSupplyCurrentLimit = Amps.of(70.0);

          public static final double kS = 0.05;
          public static final double kG = 0.4;
          public static final double kV = 6.25;
          public static final double kP = 0.75;
          public static final StaticFeedforwardSignValue kStaticFeedforwardSign =
              StaticFeedforwardSignValue.UseClosedLoopSign;
          public static final GravityTypeValue kGravityType = GravityTypeValue.Arm_Cosine;

          public static final TalonFXConfiguration kConfig =
              new TalonFXConfiguration()
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withNeutralMode(kNeutralMode)
                          .withInverted(kInverted))
                  .withFeedback(
                      new FeedbackConfigs().withSensorToMechanismRatio(kSensorToMechanismRatio))
                  .withSlot0(new Slot0Configs().withKS(kS).withKG(kG).withKV(kV).withKP(kP))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(kStatorCurrentLimit)
                          .withSupplyCurrentLimit(kSupplyCurrentLimit)
                          .withStatorCurrentLimitEnable(true)
                          .withSupplyCurrentLimitEnable(true));
        }
      }
    }

    public class Rollers { // Rollers
      public class Hardware {}

      public class Software {
        public static final Voltage kRunVoltage = Volts.of(10.0);
        public static final Voltage kBrownoutRunVoltage = Volts.of(6.0);
        public static final Voltage kReverseVoltage = Volts.of(-8.0);

        public class Config {
          public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
          public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;

          public static final Time kOpenLoopRampPeriod = Seconds.of(0.065);

          public static final Current kStatorCurrentLimit = Amps.of(60.0);
          public static final Current kSupplyCurrentLimit = Amps.of(45.0);

          public static final TalonFXConfiguration kConfig =
              new TalonFXConfiguration()
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withNeutralMode(kNeutralMode)
                          .withInverted(kInverted))
                  .withOpenLoopRamps(
                      new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(kOpenLoopRampPeriod))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(kStatorCurrentLimit)
                          .withSupplyCurrentLimit(kSupplyCurrentLimit)
                          .withStatorCurrentLimitEnable(true)
                          .withSupplyCurrentLimitEnable(true));
        }
      }
    }
  }

  public class IndexerConstants { // Indexer
    public class Hardware {}

    public class Software {
      public static final Voltage kRunVoltage = Volts.of(8.0);
      public static final Voltage kBrownoutRunVoltage = Volts.of(4.0);

      public class Config {
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
        public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;

        public static final Current kStatorCurrentLimit = Amps.of(40.0);
        public static final Current kSupplyCurrentLimit = Amps.of(35.0);

        public static final TalonFXConfiguration kConfig =
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs().withNeutralMode(kNeutralMode).withInverted(kInverted))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(kStatorCurrentLimit)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(kSupplyCurrentLimit)
                        .withSupplyCurrentLimitEnable(true));
      }
    }
  }

  public class FeederConstants { // Feeder
    public class Hardware {}

    public class Software {
      public static final Voltage kRunVoltage = Volts.of(9.0);
      public static final Voltage kBrownoutRunVoltage = Volts.of(5.0);

      public class Config {
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
        public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;

        public static final Current kStatorCurrentLimit = Amps.of(70.0);
        public static final Current kSupplyCurrentLimit = Amps.of(60.0);

        public static final TalonFXConfiguration kConfig =
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs().withNeutralMode(kNeutralMode).withInverted(kInverted))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(kStatorCurrentLimit)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(kSupplyCurrentLimit)
                        .withSupplyCurrentLimitEnable(true));
      }
    }
  }

  public class ShooterConstants { // Shooter
    public static final Distance kAutoPresetDistance = Meters.of(2.0);
    public static final Distance kTrenchPresetDistance =
        Meters.of(
            new Translation2d(Meters.of(4.125), Meters.of(0.75))
                .getDistance(FieldConstants.kHubCenterBlue.toTranslation2d()));
    public static final Distance kTowerPresetDistance =
        Meters.of(
            new Translation2d(Meters.of(1.7), Meters.of(4.0))
                .getDistance(FieldConstants.kHubCenterBlue.toTranslation2d()));
    public static final Distance kTowerLeftPresetDistance =
        Meters.of(
            new Translation2d(Meters.of(1.0), Meters.of(4.75))
                .getDistance(FieldConstants.kHubCenterBlue.toTranslation2d()));
    public static final Distance kTowerRightPresetDistance =
        Meters.of(
            new Translation2d(Meters.of(1.0), Meters.of(2.75))
                .getDistance(FieldConstants.kHubCenterBlue.toTranslation2d()));

    public class Hood { // Hood
      public class Hardware {
        public static final Angle kHoodMin = Degrees.of(10.0); // 0.1388888888888889 rot
        public static final Angle kHoodMax = Degrees.of(50.0); // 0.0277777777777778 rot
      }

      public class Software {
        public static final Angle kMaxHoodTargetError = Degrees.of(2.5);

        public class Config {
          public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
          public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;

          public static final double kSensorToMechanismRatio =
              (5.0 / 1.0) * (5.0 / 1.0) * (70.0 / 11.0);

          public static final double kP = 600.0;

          public static final AngularAcceleration kHoodAcceleration =
              RotationsPerSecondPerSecond.of(5.0);
          public static final AngularVelocity kHoodVelocity = RotationsPerSecond.of(5.0);

          public static final TalonFXConfiguration kConfig =
              new TalonFXConfiguration()
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withNeutralMode(kNeutralMode)
                          .withInverted(kInverted))
                  .withFeedback(
                      new FeedbackConfigs().withSensorToMechanismRatio(kSensorToMechanismRatio))
                  .withSlot0(new Slot0Configs().withKP(kP))
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicAcceleration(kHoodAcceleration)
                          .withMotionMagicCruiseVelocity(kHoodVelocity));
        }
      }
    }

    public class Flywheel { // Flywheel
      public class Hardware {
        public static final Transform3d kShooterOffset =
            new Transform3d(
                Units.inchesToMeters(-8.50),
                Units.inchesToMeters(0),
                Units.inchesToMeters(20.05),
                Rotation3d.kZero);
      }

      public class Software {
        public static final Frequency kFollowerUpdateFrequency = Hertz.of(1000.0); // 🤯

        public static final AngularVelocity kTestSlowVelocity = RotationsPerSecond.of(40.0);
        public static final AngularVelocity kTestFastVelocity = RotationsPerSecond.of(85.0);

        public class Config {
          public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
          public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;

          public static final double kS = 0.28;
          public static final double kV = 0.112;
          public static final double kP = 0.35;

          public static final TalonFXConfiguration kConfig =
              new TalonFXConfiguration()
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withNeutralMode(kNeutralMode)
                          .withInverted(kInverted))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(CurrentLimits.kHighStator)
                          .withSupplyCurrentLimit(CurrentLimits.kHighSupply)
                          .withStatorCurrentLimitEnable(true)
                          .withSupplyCurrentLimitEnable(true))
                  .withSlot0(new Slot0Configs().withKS(kS).withKV(kV).withKP(kP))
                  .withVoltage(new VoltageConfigs().withPeakReverseVoltage(Volts.of(0)));
        }

        public class CurrentLimits {
          public static final Current kHighStator = Amps.of(40.0);
          public static final Current kHighSupply = Amps.of(40.0);

          public static final Current kLowStator = Amps.of(7.0);
          public static final Current kLowSupply = Amps.of(6.0);
        }
      }
    }
  }

  public class VisionConstants {
    public static record PoseCameraConfig(String name, Transform3d robotToCamera, boolean front) {}

    public class Hardware {
      public static final PoseCameraConfig[] kPoseCameraConfigs = {
        new PoseCameraConfig(
            "FrontLeft",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(10.182),
                    Units.inchesToMeters(13.935),
                    Units.inchesToMeters(15.640)),
                new Rotation3d(
                    Math.toRadians(180), Math.toRadians(180 + 29.36), Math.toRadians(180))),
            true),
        new PoseCameraConfig(
            "FrontRight",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(10.182),
                    Units.inchesToMeters(-13.935),
                    Units.inchesToMeters(15.640)),
                new Rotation3d(
                    Math.toRadians(180), Math.toRadians(180 + 29.36), Math.toRadians(180))),
            true),
        new PoseCameraConfig(
            "BackLeft",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-6.334),
                    Units.inchesToMeters(16.404),
                    Units.inchesToMeters(10.278)),
                new Rotation3d(
                    Math.toRadians(180), Math.toRadians(180 + 25), Math.toRadians(-90.0 + 25.0))),
            false),
        new PoseCameraConfig(
            "BackRight",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-6.334),
                    Units.inchesToMeters(-16.404),
                    Units.inchesToMeters(10.278)),
                new Rotation3d(
                    Math.toRadians(180), Math.toRadians(180 + 25), Math.toRadians(90.0 - 25.0))),
            false),
      };
    }

    public class Software {
      public static final double BASE_XY_STD_DEV = 0.4;
      public static final double BASE_THETA_STD_DEV = Math.toRadians(25.0);

      public static final double MAX_TAG_AMBIGUITY = 0.25;
    }
  }
}
