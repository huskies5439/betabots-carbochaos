// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Pousser;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Poussoir;


public class Trajet4cubes extends SequentialCommandGroup {


  public Trajet4cubes(BasePilotable basePilotable, Poussoir poussoir) {

    Trajectory trajet = basePilotable.creerTrajectoire("4cubes_Max");

    
    addCommands(
      new InstantCommand(() -> basePilotable.resetOdometry(trajet.getInitialPose())),
      new InstantCommand(() -> basePilotable.setRamp(0)),
      new InstantCommand(() -> basePilotable.setBrake(true)),

      basePilotable.ramseteSimple(trajet),


    new Pousser(poussoir).withTimeout(0.5),
    new WaitCommand(0.5),
    new Pousser(poussoir).withTimeout(0.5),
    new WaitCommand(0.5),
    new Pousser(poussoir).withTimeout(0.5)
    );
  }
}
