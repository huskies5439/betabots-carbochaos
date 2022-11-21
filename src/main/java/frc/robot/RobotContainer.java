// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.Pousser;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Poussoir;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  
  private final BasePilotable basePilotable = new BasePilotable();
  private final Poussoir poussoir = new Poussoir();
 
  XboxController manette = new XboxController(0);

  public RobotContainer() {
    configureButtonBindings();

    basePilotable.setDefaultCommand(new RunCommand(()-> basePilotable.conduire(manette.getLeftY(), manette.getRightX()),basePilotable));
  }

  private void configureButtonBindings() {
    new JoystickButton(manette, Button.kA.value).whenHeld(new Pousser(poussoir));
  }

  public Command getAutonomousCommand() {
    return new RunCommand(()-> basePilotable.conduire(0, 0),basePilotable).withTimeout(1);
  }
}
