// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Conduire;
import frc.robot.commands.Pousser;
import frc.robot.commands.auto.S;
import frc.robot.commands.auto.Trajet2cubes;
import frc.robot.commands.auto.Trajet4cubes;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Poussoir;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  
  private final BasePilotable basePilotable = new BasePilotable();
  private final Poussoir poussoir = new Poussoir();
 
  XboxController manette = new XboxController(0);

  //trajets
  private final SendableChooser<Command> chooser = new SendableChooser<>();
  private final Command S = new S(basePilotable);
  private final Command trajet2cubes = new Trajet2cubes(basePilotable, poussoir);
  private final Command trajet4cubes = new Trajet4cubes(basePilotable, poussoir);
  public RobotContainer() {

    SmartDashboard.putData(chooser);
    chooser.setDefaultOption("Trajet Vide", new WaitCommand(14));
    chooser.addOption("Trajet en S", S);
    chooser.addOption("Trajet 2 cubes", trajet2cubes);
    chooser.addOption("Trajet 4 cubes", trajet4cubes);

    configureButtonBindings();

    basePilotable.setDefaultCommand(new Conduire(manette::getLeftY,manette::getRightX , basePilotable));
  }

  private void configureButtonBindings() {
    new JoystickButton(manette, Button.kA.value).whenHeld(new Pousser(poussoir));
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
    //return null;
      }
}
