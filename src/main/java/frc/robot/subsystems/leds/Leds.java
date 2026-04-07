package frc.robot.subsystems.leds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.MatchState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.leds.LedsIO.LED_STATE;
import frc.robot.subsystems.shooter.Shooter;

// Arduino's code | https://github.com/AlexIsCool321/1710-2026-Robot-Leds

@Logged
public class Leds {
  private final LedsIO m_io;
  private final Shooter m_shooter;
  private final Intake m_intake;

  private Timer m_autosTimer;

  public Leds(LedsIO pIo, Shooter pShooter, Intake pIntake) {
    this.m_io = pIo;
    this.m_shooter = pShooter;
    this.m_intake = pIntake;

    this.m_autosTimer = new Timer();
    this.m_autosTimer.start();
  }

  /** Runs periodic LED logic based on match state and shooter status. */
  public void periodic() {
    this.m_io.resetValue();
    
    System.out.println("PLEASE ===============");
    /*
    this.m_io.setValue(LED_STATE.DISBALED, DriverStation.isDisabled());

    this.m_io.setValue(LED_STATE.IN_AUTOS, DriverStation.isAutonomous());

    this.m_io.setValue(
        LED_STATE.AUTOS_VICTORY,
        (MatchState.autonomousWinnerIsRed.isPresent()
                ? MatchState.autonomousWinnerIsRed.get()
                : false)
            && this.m_autosTimer.get() < 3);

    this.m_io.setValue(
        LED_STATE.BROWNOUT, RobotController.isBrownedOut()); // TODO : FLASH IF CLOSE TO BROWNOUT

        
    this.m_io.setValue(LED_STATE.INTAKING, this.m_intake.getState() == IntakeStates.Intaking);

    this.m_io.setValue(LED_STATE.SHOOTING, this.m_shooter.getFPS() > 0);

    this.m_io.setValue(LED_STATE.CAN_SHOOT, false);
    */

    this.m_io.setValue(LED_STATE.DISBALED, true);

    this.m_io.update();
  }
}
