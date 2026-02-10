// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.utils.MechanismUtil.IndexerVisualSim;

@Logged
public class IndexerIOSIM implements IndexerIO {

  private final IndexerVisualSim IndexerMotorAlphaSim;
  private final IndexerVisualSim IndexerMotorBetaSim;

  @SuppressWarnings("unused")
  private final DCMotor gearbox;

  private double speed = 0.0;

  public IndexerIOSIM() {
    gearbox = DCMotor.getKrakenX60(1);
    this.IndexerMotorAlphaSim = new IndexerVisualSim("Indexer", 0.125);
    this.IndexerMotorBetaSim = new IndexerVisualSim("Indexer", 0.125);
  }

  public void setIndexMotor(double speed) {
    this.speed = speed;
  }

  public void updateVisual() {
    IndexerMotorAlphaSim.updateIndexer(speed * 20);
    Robot.telemetry().log("Indexer Alpha Speed", speed);
    SmartDashboard.putData("Indexer Alpha Visual", this.IndexerMotorAlphaSim.getMechanism());
    IndexerMotorBetaSim.updateIndexer(speed * 20);
    Robot.telemetry().log("Indexer Alpha Speed", speed);
    SmartDashboard.putData("Indexer Alpha Visual", this.IndexerMotorBetaSim.getMechanism());
  }
}
