// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

<<<<<<< HEAD:src/main/java/frc/robot/subsystems/indexer/IndexerOSIM.java
package frc.robot.subsystems.indexer;
=======
package frc.robot.subsystems.Index;
>>>>>>> 7644ef4cd8eb5deedae5ee46e79494f5155e29e4:src/main/java/frc/robot/subsystems/Index/IndexerOSIM.java

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerOSIM extends IndexerIOCTRE {
   //Creates a new IndeerOSIM. 
   private DCMotorSim sim = 
   new DCMotorSim(
    //fill in kA value later when doing PID stuff
    LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(2), 0, 0),
   DCMotor.getKrakenX44(1));
   
   private double appliedVolts = 0.0;
 

<<<<<<< HEAD:src/main/java/frc/robot/subsystems/indexer/IndexerOSIM.java
//Tells the sim to update every 0.02 seconds with data of the mtors position and velocity.
=======

>>>>>>> 7644ef4cd8eb5deedae5ee46e79494f5155e29e4:src/main/java/frc/robot/subsystems/Index/IndexerOSIM.java
private void updateInputs(IndexerIOInputs inputs) {
   sim.setInputVoltage(appliedVolts);
   sim.update(0.02);
   inputs.position = sim.getAngularPositionRad();
   inputs.velocity = sim.getAngularVelocityRadPerSec();
   inputs.appliedVolts = appliedVolts;
} 
}

