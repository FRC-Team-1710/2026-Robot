package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Shooter extends SubsystemBase {

  public enum SHOOTER_STATE {
    STOP(RotationsPerSecond.of(0)),

    IDLE(RotationsPerSecond.of(200)),
    SHOOT(RotationsPerSecond.of(250)),

    PRESET_PASS(RotationsPerSecond.of(100)),
    PRESET_SHOOT(RotationsPerSecond.of(250));


    private final AngularVelocity m_velocity;

    SHOOTER_STATE(AngularVelocity velocity) {
      this.m_velocity = velocity;
    }
  };

  private SHOOTER_STATE m_state;

  private final ShooterIO m_io;


  public Shooter(ShooterIO io) {
    this.m_io = io;
    this.m_state = SHOOTER_STATE.STOP;
  }

  @Override
  public void periodic() {
    switch (this.m_state) {
      case STOP:
        this.m_io.stop();
        break;


      case IDLE:
        this.m_io.setVelocity(SHOOTER_STATE.IDLE.m_velocity);
        break;

      case SHOOT:
        this.m_io.setVelocity(SHOOTER_STATE.SHOOT.m_velocity);
        break;


        case PRESET_PASS:
        this.m_io.setVelocity(SHOOTER_STATE.PRESET_PASS.m_velocity);
        break;
        
      case PRESET_SHOOT:
        this.m_io.setVelocity(SHOOTER_STATE.PRESET_SHOOT.m_velocity);
        break;
    }
  }


  public void setState(SHOOTER_STATE state) {
    this.m_state = state;
  }

  public SHOOTER_STATE getState() {
    return this.m_state;
  }
}