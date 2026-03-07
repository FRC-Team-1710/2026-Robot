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

  private final IndexerVisualSim m_indexerMotorAlphaSim;
  private final IndexerVisualSim m_indexerMotorBetaSim;

  @SuppressWarnings("unused")
  private final DCMotor m_gearbox;

  private double m_speed = 0.0;

  public IndexerIOSIM() {
    m_gearbox = DCMotor.getKrakenX60(1);
    this.m_indexerMotorAlphaSim = new IndexerVisualSim("Indexer", 0.125);
    this.m_indexerMotorBetaSim = new IndexerVisualSim("Indexer", 0.125);
  }

  public void setIndexMotor(double speed) {
    this.m_speed = speed;
  }

  public void update() {
    m_indexerMotorAlphaSim.updateIndexer(m_speed * 20);
    Robot.telemetry().log("Indexer Alpha Speed", m_speed);
    SmartDashboard.putData("Indexer Alpha Visual", this.m_indexerMotorAlphaSim.getMechanism());
    m_indexerMotorBetaSim.updateIndexer(m_speed * 20);
    Robot.telemetry().log("Indexer Beta Speed", m_speed);
    SmartDashboard.putData("Indexer Beta Visual", this.m_indexerMotorBetaSim.getMechanism());
  }
}
