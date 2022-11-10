// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Poussoir extends SubsystemBase {
  private DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  
  public Poussoir() {
    rentrer();
  }

  @Override
  public void periodic() {}

  public void rentrer() {
    piston.set(Value.kForward);
  }

  public void sortir() {
    piston.set(Value.kReverse);
  }

}
