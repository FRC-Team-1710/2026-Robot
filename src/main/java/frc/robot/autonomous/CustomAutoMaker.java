package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedStates;
import java.lang.reflect.Field;

public class CustomAutoMaker {
  private Command customAuto = Commands.none();
  private final SendableChooser<Translation2d> poseChooser = new SendableChooser<>();
  private final SendableChooser<Command> commandChooser = new SendableChooser<>();

  private final Trigger submitTrigger =
      new Trigger(() -> SmartDashboard.getBoolean("Auto/Submit", false));
  private final Trigger resetTrigger =
      new Trigger(() -> SmartDashboard.getBoolean("Auto/Reset", false));
  private final Trigger updateTrigger =
      new Trigger(() -> SmartDashboard.getBoolean("Auto/Update", false));

  public CustomAutoMaker(Superstructure superstructure) {
    poseChooser.setDefaultOption("Custom", new Translation2d());

    SmartDashboard.putBoolean("Auto/Submit", false);
    SmartDashboard.putBoolean("Auto/Reset", false);
    SmartDashboard.putNumber("Auto/CustomTheta(deg)", 0);
    SmartDashboard.putNumber("Auto/CustomX", 0);
    SmartDashboard.putNumber("Auto/CustomY", 0);
    SmartDashboard.putData("Auto/PoseChooser", poseChooser);
    SmartDashboard.putData("Auto/CommandChooser", commandChooser);
    SmartDashboard.putBoolean("Auto/AddPath", true);
    SmartDashboard.putBoolean("Auto/Update", false);

    for (WantedStates state : WantedStates.values()) {
      if (state.name().contains("Auto")) {
        commandChooser.addOption(state.name(), superstructure.setWantedStateCommand(state));
      }
    }

    var fields = FieldConstants.AutoConstants.class.getFields();

    for (Field field : fields) {
      try {
        // No field needed because it's static
        Object obj = field.get(null);
        if (obj instanceof Translation2d) {
          Translation2d translation = (Translation2d) obj;
          poseChooser.addOption(field.getName(), translation);
        }
      } catch (Exception e) {
        DriverStation.reportError(
            "Failed to get field " + field.getName() + " in CustomAutoMaker", e.getStackTrace());
      }
    }

    updateTrigger.onTrue(
        Commands.runOnce(
                () -> {
                  Robot.telemetry()
                      .log(
                          "SmartDashboard/Auto/CustomPoseViewer",
                          new Pose2d(
                              SmartDashboard.getNumber("Auto/CustomX", 0),
                              SmartDashboard.getNumber("Auto/CustomY", 0),
                              Rotation2d.fromDegrees(
                                  SmartDashboard.getNumber("Auto/CustomTheta(deg)", 0))),
                          Pose2d.struct);
                  SmartDashboard.putBoolean("Auto/Update", false);
                })
            .ignoringDisable(true));
    submitTrigger.onTrue(
        Commands.runOnce(
                () -> {
                  if (SmartDashboard.getBoolean("Auto/AddPath", true)) {
                    Translation2d translationToSet = poseChooser.getSelected();
                    if (translationToSet == new Translation2d()) {
                      translationToSet =
                          new Translation2d(
                              SmartDashboard.getNumber("Auto/CustomX", 0),
                              SmartDashboard.getNumber("Auto/CustomY", 0));
                    }
                    customAuto =
                        Commands.sequence(
                            customAuto,
                            AutoPathBuilder.getBuilder()
                                .build(
                                    new Path(
                                        new Waypoint(
                                            new Pose2d(
                                                translationToSet,
                                                Rotation2d.fromDegrees(
                                                    SmartDashboard.getNumber(
                                                        "Auto/CustomTheta(deg)", 0)))))));
                  } else {
                    customAuto = Commands.sequence(customAuto, commandChooser.getSelected());
                  }
                  SmartDashboard.putBoolean("Auto/Submit", false);
                })
            .ignoringDisable(true));
    resetTrigger.onTrue(
        Commands.runOnce(
                () -> {
                  customAuto = Commands.none();
                  SmartDashboard.putBoolean("Auto/Reset", false);
                })
            .ignoringDisable(true));
  }

  public Command getAuto() {
    return customAuto;
  }
}
