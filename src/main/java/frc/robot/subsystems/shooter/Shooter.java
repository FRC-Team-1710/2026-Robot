package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;

@Logged
public class Shooter {

  private ShooterState state;

  private final ShooterIO io;

  private final TimesConsumer timesConsumer;

  private AngularVelocity velocity;
  private Angle hoodAngle;

  public Shooter(ShooterIO io, TimesConsumer consumer) {

    this.io = io;
    this.timesConsumer = consumer;
    this.state = ShooterState.Stop;

    this.velocity = RotationsPerSecond.of(0);
    this.hoodAngle = Degrees.of(0);
  }

  public void periodic() {
    switch (this.state) {
      case Shoot:
        this.io.setTargetVelocity(this.getTargetVelocity());
        this.io.setHoodAngle(this.getTargetHoodAngle());
        break;

      default:
        this.io.setTargetVelocity(this.state.getVelocity());
        this.io.setHoodAngle(this.state.getHoodAngle());
        break;
    }

    // Stop motor if velocity is 0
    if (this.getTargetVelocity().in(DegreesPerSecond) == 0) {
      this.io.stop();
    }

    this.io.update();
  }

  public void setVelocity(AngularVelocity velocity) {
    this.velocity = velocity;
  }

  public AngularVelocity getVelocity() {
    return this.io.getVelocity();
  }

  public AngularVelocity getTargetVelocity() {
    return this.velocity;
  }

  public void setTargetHoodAngle(Angle angle) {
    this.hoodAngle = angle;
  }

  public Angle getTargetHoodAngle() {
    return this.hoodAngle;
  }

  public enum ShooterState {
    Stop(Milliseconds.of(60), RotationsPerSecond.of(0), Degrees.of(0)),
    Idle(Milliseconds.of(60), RotationsPerSecond.of(200), Degrees.of(0)),
    Shoot(Milliseconds.of(20), RotationsPerSecond.of(250), Degrees.of(0)),
    PresetPass(Milliseconds.of(20), RotationsPerSecond.of(100), Degrees.of(0)),
    PresetShoot(Milliseconds.of(20), RotationsPerSecond.of(250), Degrees.of(0));

    private final Time subsystemPeriodicFrequency;
    private final AngularVelocity velocity;
    private final Angle hoodAngle;

    ShooterState(Time subsystemPeriodicFrequency, AngularVelocity velocity, Angle hoodAngle) {
      this.subsystemPeriodicFrequency = subsystemPeriodicFrequency;
      this.velocity = velocity;
      this.hoodAngle = hoodAngle;
    }

    Time getSubsystemPeriodicFrequency() {
      return this.subsystemPeriodicFrequency;
    }

    AngularVelocity getVelocity() {
      return this.velocity;
    }

    Angle getHoodAngle() {
      return this.hoodAngle;
    }
  };

  public void setState(ShooterState state) {
    if (!this.state
        .getSubsystemPeriodicFrequency()
        .isEquivalent(state.getSubsystemPeriodicFrequency())) {
      timesConsumer.accept(Subsystems.Shooter, state.getSubsystemPeriodicFrequency());
    }
    this.state = state;
  }

  public ShooterState getState() {
    return this.state;
  }
}
