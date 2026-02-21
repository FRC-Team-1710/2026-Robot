package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.MatchState;
import frc.robot.subsystems.leds.LedsIO.LED_STATE;
import frc.robot.subsystems.shooter.Shooter;

// Arduino's code | https://github.com/AlexIsCool321/1710-2026-Robot-Leds

@Logged
public class Leds {
  private final LedsIO m_io;
  private final Shooter m_shooter;

  private Timer m_autosTimer;

  public Leds(LedsIO pIo, Shooter pShooter) {
    this.m_io = pIo;
    this.m_shooter = pShooter;

    this.m_autosTimer = new Timer();
    this.m_autosTimer.start();
  }

  public void periodic() {
    this.m_io.setValue(LED_STATE.ATTACKING, MatchState.timeTillActive().in(Seconds) <= 3);
    this.m_io.setValue(
        LED_STATE.BROWOUT, RobotController.isBrownedOut()); // TODO : FLASH IF CLOSE TO BROWNOUT

    boolean autosVictory =
        MatchState.autonomousWinnerIsRed.isPresent()
            ? MatchState.autonomousWinnerIsRed.get()
            : false;

    this.m_io.setValue(LED_STATE.AUTOS, autosVictory && this.m_autosTimer.get() < 3);

    double shooterCharge =
        (this.m_shooter.getVelocity().magnitude() / this.m_shooter.getTargetVelocity().magnitude())
            * 100;
    this.m_io.setFlyWheelCharge(shooterCharge);

    this.m_io.update();
  }
}
