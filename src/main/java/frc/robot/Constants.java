// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {

    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.63); //A verifier changer argument kinematic
    public static final double kSRamsete = 0.6;
    public static final double kVRamsete = 4.48;
    public static final double kPRamsete = 2;
    public static final double rampTeleop = 0.2;
}
