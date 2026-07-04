package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.Alliance;
import frc.robot.constants.MatchState;
import frc.robot.subsystems.Superstructure;

// Arduino's code | https://github.com/AlexIsCool321/1710-2026-Robot-Leds

public class Leds {
  private SPI spi;

  Timer timer = new Timer();
  private final Superstructure m_superstructure;

  public Leds(Superstructure superstructure) {
    this.m_superstructure = superstructure;

    spi = new SPI(SPI.Port.kOnboardCS0);
    spi.setClockRate(500000); // 500 kHz safe start
    spi.setMode(SPI.Mode.kMode3);

    spi.setChipSelectActiveLow();

    timer.stop();
    timer.reset();

    SmartDashboard.putBoolean("Leds/manual", false);
    SmartDashboard.putNumber("Leds/manualNum", 0);
  }

  private Integer commandValue = 0;

  /** Priority booleans for the LEDs */
  public Boolean[] inputBooleans = {false, false, false, false, false, false, false, false};

  public void periodic() {
    set();
    encoder();
    sendData(commandValue);
  }

  /** Sets the input booleans based on the current state of the robot */
  private void set() {
    this.inputBooleans[0] = MatchState.autonomousWinnerIsRed.isPresent() && !timer.hasElapsed(3);

    if (MatchState.autonomousWinnerIsRed.isPresent()) {
      if (!timer.isRunning()) {
        timer.start();
      }

      this.inputBooleans[1] =
          MatchState.autonomousWinnerIsRed.get() ? Alliance.redAlliance : !Alliance.redAlliance;
    }

    this.inputBooleans[2] =
        this.m_superstructure.isStateTryingToShoot()
            && this.m_superstructure.flywheelAtTarget()
            && this.m_superstructure.driveAtTarget();

    this.inputBooleans[3] =
        this.m_superstructure.isStateTryingToShoot()
            && (!this.m_superstructure.flywheelAtTarget()
                || !this.m_superstructure.driveAtTarget());

    this.inputBooleans[4] = this.m_superstructure.currentStateUsesIntake();

    this.inputBooleans[5] = RobotController.isBrownedOut();

    this.inputBooleans[6] = !DriverStation.isDSAttached();

    this.inputBooleans[7] = DriverStation.isDisabled();

    for (int i = 0; i < inputBooleans.length; i++) {
      Logger.recordOutput("LEDBooleans/" + i, inputBooleans[i]);
    }
  }

  private void encoder() {
    if (!SmartDashboard.getBoolean("Leds/manual", false)) {
      if (inputBooleans[0]) {
        commandValue = inputBooleans[1] ? 0 : 1;
        return;
      }

      for (int i = 2; i < inputBooleans.length; i++) {
        if (inputBooleans[i]) {
          commandValue = i;
          break;
        }
      }
    } else {
      commandValue = (int) SmartDashboard.getNumber("Leds/manualNum", 0);
    }
  }

  private void sendData(int value) {
    byte[] data = new byte[1];
    byte[] rx = new byte[1];
    Logger.recordOutput("LEDBooleans/CommandValue", value);
    data[0] = (byte) (value & 0xFF);
    spi.transaction(data, rx, 1);
  }
}