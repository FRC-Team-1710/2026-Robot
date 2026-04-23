package frc.robot.subsystems.leds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.MatchState;
import frc.robot.subsystems.Superstructure;

// Arduino's code | https://github.com/AlexIsCool321/1710-2026-Robot-Leds

@Logged
public class Leds {
  /** Creates a new LEDSubsystem. */
  // private SerialPort uart;

  private SPI spi;

  Timer timer = new Timer();
  private Superstructure m_superstructure;

  public Leds() {
    // uart = new SerialPort(4800, SerialPort.Port.kUSB1);
    spi = new SPI(SPI.Port.kOnboardCS0);
    spi.setClockRate(500000); // 500 kHz safe start
    spi.setMode(SPI.Mode.kMode3);

    spi.setChipSelectActiveLow();
  }

  private Integer commandValue = 0;

  /** Priority booleans for the LEDs */
  // Autonomous?, Can shoot?, Can't shoot?, Intaking?, Won auto?, Brownout?, Disabled?,
  // Disconnected?
  public Boolean[] inputBooleans = {false, false, false, false, false, false, false, false};

  public void periodic() {
    set(); // Getting condition of robot
    encoder(); // Setting the command value
    sendData(commandValue); // Send Phase
  }

  /** Sets the input booleans based on the current state of the robot */
  private void set() { // Decimal phase
    // In auto lights
    this.inputBooleans[0] = DriverStation.isAutonomous();

    // Can shoot lights
    this.inputBooleans[1] =
        this.m_superstructure.getCurrentState() == Superstructure.CurrentStates.Shoot
            && this.m_superstructure.flywheelAtTarget();

    // Can't shoot lights
    this.inputBooleans[2] =
        this.m_superstructure.getCurrentState() == Superstructure.CurrentStates.Shoot
            && !this.m_superstructure.flywheelAtTarget();

    // Intake lights
    this.inputBooleans[3] = this.m_superstructure.currentStateUsesIntake();

    // Won in auto lights TODO add a timer to this so that it only lasts for a second or two after
    // auto
    this.inputBooleans[4] =
        (MatchState.autonomousWinnerIsRed.isPresent()
            ? MatchState.autonomousWinnerIsRed.get()
            : false);

    // Brownout lights
    this.inputBooleans[5] = RobotController.isBrownedOut();

    // Disabled lights
    this.inputBooleans[6] = DriverStation.isDisabled();

    // Disconnected lights
    this.inputBooleans[7] = !DriverStation.isDSAttached();

    SmartDashboard.putBooleanArray("Input Booleans", inputBooleans);
  }

  /** Sets the input booleans to send based on the priorities of the states */
  private void encoder() { // Transition phase
    for (int i = 0;
        i < inputBooleans.length;
        i++) { // Picks the first true sequence based on priority
      if (inputBooleans[i]) {
        commandValue = i;
        break;
      }
    }
  }

  /** Sends the command value to the LEDs */
  private void sendData(int value) { // Sending data (duh)
    byte[] data = new byte[1]; // Create a byte array of length 1
    byte[] rx = new byte[1];
    SmartDashboard.putNumber("LED Data", value);
    data[0] =
        (byte) (value & 0xFF); // Store value as byte in the array, and mask to ensure unsigned byte
    // System.out.println("Sending Data: " + value); // Print the data
    // uart.write(data, data.length); // Write the byte array to the serial port
    spi.transaction(data, rx, 1); // Write the byte array to the SPI port
  }

  /** Sets the superstructure for the LED subsystem */
  public void setSuperstructure(Superstructure superstructure) {
    this.m_superstructure = superstructure;
  }
}
