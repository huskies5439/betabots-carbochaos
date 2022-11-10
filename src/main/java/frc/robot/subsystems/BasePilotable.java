// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class BasePilotable extends SubsystemBase {
  
  private WPI_TalonFX moteurGauche = new WPI_TalonFX(1);
  private WPI_TalonFX moteurDroit = new WPI_TalonFX(3);

  private DifferentialDrive drive = new DifferentialDrive(moteurGauche, moteurDroit);

  public BasePilotable() {}

  public void conduire(double vx,double vz) {
    //vx la vitesse pour avancer et vz la vitesse pour tourner
    drive.arcadeDrive(-0.8*vx, 0.5*vz);
  }

  public void autoConduire(double voltGauche, double voltDroit) {
    //Fonction conduire pour en Autonomous 
    moteurGauche.setVoltage(voltGauche);
    moteurDroit.setVoltage(voltDroit);
    drive.feed();
  }

  public void stop() {
    //Bein Le Robot Bein y s'arrête
    autoConduire(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
