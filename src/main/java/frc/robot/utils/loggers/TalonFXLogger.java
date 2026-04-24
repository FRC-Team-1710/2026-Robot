package frc.robot.utils.loggers;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {
  public TalonFXLogger() {
    super(TalonFX.class);
  }

  @Override
  public void update(EpilogueBackend backend, TalonFX talon) {
    // Refresh signals that aren't automatically refreshed in the IO implementation due to them not
    // being needed
    if (Epilogue.shouldLog(Importance.INFO)) {
      backend.log("ClosedLoopError", talon.getClosedLoopError(true).getValue());
    }

    // Don't refresh signals that are updated in the io
    backend.log("Velocity", talon.getVelocity(false).getValue());
    backend.log("Position", talon.getPosition(false).getValue());
    backend.log("StatorCurrent", talon.getStatorCurrent(false).getValue());
    backend.log("SupplyCurrent", talon.getSupplyCurrent(false).getValue());
    backend.log("MotorVoltage", talon.getMotorVoltage(false).getValue());
    backend.log("ClosedLoopReference", talon.getClosedLoopReference(false).getValue());
  }
}
