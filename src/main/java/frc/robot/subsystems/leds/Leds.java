package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.constants.Alliance;
import frc.robot.constants.MatchState;
import frc.robot.subsystems.Superstructure;

// Arduino's code | https://github.com/AlexIsCool321/1710-2026-Robot-Leds

public class Leds {

  private SPI spi;

  private final Timer timer = new Timer();

  private final Superstructure m_superstructure;

  public Leds(Superstructure superstructure) {
    m_superstructure = superstructure;

    spi = new SPI(SPI.Port.kOnboardCS0);
    spi.setClockRate(500000); // 500 kHz safe start
    spi.setMode(SPI.Mode.kMode3);

    spi.setChipSelectActiveLow();

    timer.stop();
    timer.reset();
  }

  private Integer commandValue = 0;

  /** Priority booleans for the LEDs */
  public Boolean[] inputBooleans = {false, false, false, false, false, false, false, false};

  public void periodic() {
    set(); // Getting condition of robot
    encoder(); // Setting the command value
    sendData(commandValue); // Send Phase
  }

  /** Sets the input booleans based on the current state of the robot */
  private void set() {
    // Won in auto is present
    inputBooleans[0] = MatchState.autonomousWinnerIsRed.isPresent() && !timer.hasElapsed(3);

    if (MatchState.autonomousWinnerIsRed.isPresent()) {
      if (!timer.isRunning()) {
        timer.start();
      }

      // Won in auto lights
      inputBooleans[1] =
          MatchState.autonomousWinnerIsRed.get() ? Alliance.redAlliance : !Alliance.redAlliance;
    }

    // Can shoot lights
    inputBooleans[2] = m_superstructure.isStateTryingToShoot() && m_superstructure.readyToShoot();

    // Can't shoot lights
    inputBooleans[3] = !inputBooleans[2];

    // Intake lights
    inputBooleans[4] = m_superstructure.currentStateUsesIntake();

    // Brownout lights
    inputBooleans[5] = RobotController.isBrownedOut();

    // Disconnected lights
    inputBooleans[6] = !DriverStation.isDSAttached();

    // Disabled lights
    inputBooleans[7] = DriverStation.isDisabled();
  }

  /** Sets the input booleans to send based on the priorities of the states */
  private void encoder() {
    if (inputBooleans[0]) { // If won in auto is present, it takes priority over everything else
      commandValue = inputBooleans[1] ? 0 : 1;
      return;
    }

    for (int i = 2;
        i < inputBooleans.length;
        i++) { // Picks the first true sequence based on priority
      if (inputBooleans[i]) {
        commandValue = i;
        break;
      }
    }
  }

  /** Sends the command value to the LEDs */
  private void sendData(int value) {
    byte[] data = new byte[1]; // Create a byte array of length 1
    byte[] rx = new byte[1];
    Robot.telemetry().log("LEDBooleans/CommandValue", value);
    data[0] =
        (byte) (value & 0xFF); // Store value as byte in the array, and mask to ensure unsigned byte
    spi.transaction(data, rx, 1); // Write the byte array to the SPI port
  }
}
