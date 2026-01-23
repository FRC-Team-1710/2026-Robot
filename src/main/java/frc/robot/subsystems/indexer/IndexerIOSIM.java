// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.




package frc.robot.subsystems.indexer;
<<<<<<< HEAD:src/main/java/frc/robot/subsystems/indexer/IndexerOSIM.java
=======
package frc.robot.subsystems.Index;
>>>>>>> 7644ef4 (added some code for index sim, such as returning motor positions, but still a work in progress and not yet fully functioning.):src/main/java/frc/robot/subsystems/Index/IndexerOSIM.java
=======

>>>>>>> 1a14fdc (fixed merge conflicts):src/main/java/frc/robot/subsystems/indexer/IndexerIOSIM.java

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

<<<<<<< HEAD:src/main/java/frc/robot/subsystems/indexer/IndexerOSIM.java
<<<<<<< HEAD:src/main/java/frc/robot/subsystems/indexer/IndexerOSIM.java
//Tells the sim to update every 0.02 seconds with data of the mtors position and velocity.
=======

>>>>>>> 7644ef4 (added some code for index sim, such as returning motor positions, but still a work in progress and not yet fully functioning.):src/main/java/frc/robot/subsystems/Index/IndexerOSIM.java
=======
>>>>>>> 1a14fdc (fixed merge conflicts):src/main/java/frc/robot/subsystems/indexer/IndexerIOSIM.java
private void updateInputs(IndexerIOInputs inputs) {
   sim.setInputVoltage(appliedVolts);
   sim.update(0.02);
   inputs.position = sim.getAngularPositionRad();
   inputs.velocity = sim.getAngularVelocityRadPerSec();
   inputs.appliedVolts = appliedVolts;
} 
}

