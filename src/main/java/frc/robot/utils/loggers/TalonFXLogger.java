package frc.robot.utils.loggers;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {
  public TalonFXLogger() {
    super(TalonFX.class);
  }

  @Override
  public void update(EpilogueBackend backend, TalonFX talon) {
    backend.log("Id", talon.getDeviceID());
    backend.log("ClosedLoopReference", talon.getClosedLoopReference().getValue());
    backend.log("ClosedLoopError", talon.getClosedLoopError().getValue());
    backend.log("IsProLicensed", talon.getIsProLicensed().getValue());
    backend.log("MotorVoltage", talon.getMotorVoltage().getValue());
    backend.log("Velocity", talon.getVelocity().getValue());
    backend.log("StatorCurrent", talon.getStatorCurrent().getValue());
    backend.log("Position", talon.getPosition().getValue());
  }
}
