// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.




package frc.robot.subsystems.indexer;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSIM extends IndexerIOCTRE {
   //Creates a new IndeerOSIM. 
   private DCMotorSim sim = 
   new DCMotorSim(
    //fill in kA value later when doing PID stuff
    LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(2), 0, 0),
   DCMotor.getKrakenX44(1));
   
   private double appliedVolts = 0.0;

private void updateInputs(IndexerIOInputs inputs) {
   sim.setInputVoltage(appliedVolts);
   sim.update(0.02);
   inputs.position = sim.getAngularPositionRad();
   inputs.velocity = sim.getAngularVelocityRadPerSec();
   inputs.appliedVolts = appliedVolts;
} 
}

