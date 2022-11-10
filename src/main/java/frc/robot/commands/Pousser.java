// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Poussoir;

public class Pousser extends CommandBase {
  Poussoir poussoir;

  public Pousser(Poussoir poussoir) {
    this.poussoir = poussoir;
    addRequirements(poussoir);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    poussoir.sortir();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    poussoir.rentrer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
