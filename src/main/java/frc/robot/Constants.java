// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants { //TO DO changer les données
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.66); 
    public static final double kVRamsete = 4.6694;
    public static final double kSRamsete = 0.62455;
    public static final double kARamsete = 0.28556;
    public static final double kPRamsete = 4.8954;
    public static final double kRampTeleop = 0.2;
}